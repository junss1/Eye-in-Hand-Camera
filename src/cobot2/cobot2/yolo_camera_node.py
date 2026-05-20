# yolo_camera_node v0.600 2026-02-05
# [이번 버전에서 수정된 사항]
# - (기능구현) /follow/enable(Bool) 구독 추가: enable=False면 YOLO 추론/락온/error_norm/lock_done 발행을 중지(충돌 방지)
# - (기능구현) enable=False 전환 시 lock 상태 리셋 + lock_done(False) 1회 publish(TRANSIENT_LOCAL 라치값 클리어 목적)
# - (유지) UI 토픽(annotated/ui_event) 퍼블리시 및 기존 마스크 기반 aim/EMA/락온/lock_done 게이트 로직 유지

"""
YOLO 기반 객체 탐지 + 락온 + error_norm 퍼블리시 노드 (eye-in-hand tracking)

주요 토픽
- Sub:  image_topic (RGB/IR)
- Pub:  /follow/error_norm (Float32MultiArray [ex, ey])
- Pub:  /follow/lock_done (Bool, TRANSIENT_LOCAL; late-joiner safe)
- Pub:  /follow/annotated_image (Image, 옵션)
- Pub:  /follow/ui_event (String)

추가(v0.600)
- Sub: /follow/enable (Bool)
  - False면 추론/락온/lock_done/error_norm(입력)을 사실상 차단하여
    follow OFF 상태에서 salute/shoot 등의 동작과 충돌하지 않도록 한다.
"""

from __future__ import annotations

import datetime
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, String
from ultralytics import YOLO

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None  # type: ignore[assignment]


@dataclass
class Detection:
    class_id: int
    class_name: str
    confidence: float
    x1_px: float
    y1_px: float
    x2_px: float
    y2_px: float
    track_id: Optional[int] = None
    contour_xy: Optional[np.ndarray] = None  # shape (N,2), float32

    @property
    def bbox_center_px(self) -> Tuple[float, float]:
        return ((self.x1_px + self.x2_px) * 0.5, (self.y1_px + self.y2_px) * 0.5)

    @property
    def area(self) -> float:
        return max(0.0, self.x2_px - self.x1_px) * max(0.0, self.y2_px - self.y1_px)


def _compute_error_norm(center_px: Tuple[float, float], w: int, h: int) -> Tuple[float, float]:
    cx, cy = center_px
    ex = (cx - (w * 0.5)) / (w * 0.5)
    ey = (cy - (h * 0.5)) / (h * 0.5)
    ex = float(np.clip(ex, -1.0, 1.0))
    ey = float(np.clip(ey, -1.0, 1.0))
    return ex, ey


def _resolve_model_path(model: str, package_name: str = "cobot2") -> Tuple[str, List[str]]:
    tried: List[str] = []
    model = str(model).strip()
    if not model:
        return model, tried

    tried.append(model)
    if os.path.isabs(model) and os.path.isfile(model):
        return model, tried
    if os.path.isfile(model):
        return os.path.abspath(model), tried

    here_dir = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.join(here_dir, model)
    tried.append(cand)
    if os.path.isfile(cand):
        return cand, tried

    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory(package_name)
            cand2 = os.path.join(share_dir, model)
            tried.append(cand2)
            if os.path.isfile(cand2):
                return cand2, tried

            cand3 = os.path.join(share_dir, "weights", model)
            tried.append(cand3)
            if os.path.isfile(cand3):
                return cand3, tried
        except Exception:
            pass

    for guess in [
        os.path.expanduser(f"~/cobot_ws/src/{package_name}/{package_name}/{model}"),
        os.path.expanduser(f"~/cobot_ws/src/{package_name}/{model}"),
    ]:
        tried.append(guess)
        if os.path.isfile(guess):
            return guess, tried

    return model, tried


def _centroid_from_contour(contour_xy: np.ndarray) -> Optional[Tuple[int, int, int]]:
    """
    contour_xy: (N,2) float -> returns (cx, cy, top_y) as ints
    """
    if contour_xy is None or len(contour_xy) < 3:
        return None
    cnt = contour_xy.astype(np.int32)
    M = cv2.moments(cnt)
    if M.get("m00", 0.0) == 0.0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    top_y = int(np.min(cnt[:, 1]))
    return cx, cy, top_y


class YoloCameraNode(Node):
    def __init__(self):
        super().__init__("yolo_camera_node")
        self._bridge = CvBridge()

        # -------------------------------
        # Parameters
        # -------------------------------
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("model", "night.pt")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("target_class_name", "person")

        # 락온/유지 threshold
        self.declare_parameter("lock_conf_high", 0.93)
        self.declare_parameter("maintain_conf_low", 0.4)
        self.declare_parameter("lost_timeout_sec", 0.6)

        # lock_done error threshold
        self.declare_parameter("lock_done_error_thresh", 0.15)

        # tracker
        self.declare_parameter("use_tracker", True)
        self.declare_parameter("tracker_yaml", "bytetrack.yaml")
        self.declare_parameter("retina_masks", True)

        # 출력
        self.declare_parameter("publish_topic", "/follow/error_norm")
        self.declare_parameter("show_debug", False)

        # UI / Debug output topics
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("annotated_topic", "/follow/annotated_image")
        self.declare_parameter("ui_event_topic", "/follow/ui_event")

        # 입력 뒤집기
        self.declare_parameter("input_flip_v", True)
        self.declare_parameter("input_flip_h", False)

        # 락온 완료 토픽
        self.declare_parameter("lock_done_topic", "/follow/lock_done")
        self.declare_parameter("lock_done_delay_sec", 1.0)

        # 시간 기반 토픽 전환
        self.declare_parameter("enable_time_based_switch", True)
        self.declare_parameter("day_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("night_image_topic", "/camera/camera/infra1/image_rect_raw")
        self.declare_parameter("day_start_hms", [7, 30, 0])
        self.declare_parameter("night_start_hms", [20, 41, 0])
        self.declare_parameter("time_check_period_sec", 1.0)

        # 외부 override 토픽
        self.declare_parameter("image_topic_override_topic", "/follow/image_topic_override")

        # Mask centroid mode
        self.declare_parameter("use_mask_centroid", True)
        self.declare_parameter("mask_aim_up_ratio", 0.10)
        self.declare_parameter("mask_draw_contour", True)

        # Aim EMA filter
        self.declare_parameter("aim_ema_enable", True)
        self.declare_parameter("aim_ema_alpha", 0.25)  # 0.15~0.35 추천

        # Debug draw controls
        self.declare_parameter("debug_draw_bbox", False)  # 기본: bbox 숨김
        self.declare_parameter("debug_draw_mask", True)   # 기본: 마스크(윤곽) 표시

        # (NEW) Follow enable gate
        self.declare_parameter("follow_enable_topic", "/follow/enable")
        self.declare_parameter("follow_enable_default", True)

        # -------------------------------
        # Read params
        # -------------------------------
        self._image_topic: str = str(self.get_parameter("image_topic").value)
        self._model_path_raw: str = str(self.get_parameter("model").value)
        self._imgsz: int = int(self.get_parameter("imgsz").value)
        self._target_class: str = str(self.get_parameter("target_class_name").value)

        self._lock_conf_high: float = float(self.get_parameter("lock_conf_high").value)
        self._maintain_conf_low: float = float(self.get_parameter("maintain_conf_low").value)
        self._lost_timeout_sec: float = float(self.get_parameter("lost_timeout_sec").value)
        self._lock_done_err_th: float = float(self.get_parameter("lock_done_error_thresh").value)

        self._use_tracker: bool = bool(self.get_parameter("use_tracker").value)
        self._tracker_yaml: str = str(self.get_parameter("tracker_yaml").value)
        self._retina_masks: bool = bool(self.get_parameter("retina_masks").value)

        self._publish_topic: str = str(self.get_parameter("publish_topic").value)
        self._show_debug: bool = bool(self.get_parameter("show_debug").value)
        self._publish_annotated: bool = bool(self.get_parameter("publish_annotated").value)
        self._annotated_topic: str = str(self.get_parameter("annotated_topic").value)
        self._ui_event_topic: str = str(self.get_parameter("ui_event_topic").value)

        self._input_flip_v: bool = bool(self.get_parameter("input_flip_v").value)
        self._input_flip_h: bool = bool(self.get_parameter("input_flip_h").value)

        self._lock_done_topic: str = str(self.get_parameter("lock_done_topic").value)
        self._lock_done_delay_sec: float = float(self.get_parameter("lock_done_delay_sec").value)

        self._enable_time_switch: bool = bool(self.get_parameter("enable_time_based_switch").value)
        self._day_topic: str = str(self.get_parameter("day_image_topic").value)
        self._night_topic: str = str(self.get_parameter("night_image_topic").value)
        self._day_hms: List[int] = list(self.get_parameter("day_start_hms").value)
        self._night_hms: List[int] = list(self.get_parameter("night_start_hms").value)
        self._time_check_period: float = float(self.get_parameter("time_check_period_sec").value)

        self._override_topic: str = str(self.get_parameter("image_topic_override_topic").value)

        self._use_mask_centroid: bool = bool(self.get_parameter("use_mask_centroid").value)
        self._mask_aim_up_ratio: float = float(self.get_parameter("mask_aim_up_ratio").value)
        self._mask_draw_contour: bool = bool(self.get_parameter("mask_draw_contour").value)

        self._aim_ema_enable: bool = bool(self.get_parameter("aim_ema_enable").value)
        self._aim_ema_alpha: float = float(self.get_parameter("aim_ema_alpha").value)

        self._debug_draw_bbox: bool = bool(self.get_parameter("debug_draw_bbox").value)
        self._debug_draw_mask: bool = bool(self.get_parameter("debug_draw_mask").value)

        self._follow_enable_topic: str = str(self.get_parameter("follow_enable_topic").value)
        self._follow_enable: bool = bool(self.get_parameter("follow_enable_default").value)

        # -------------------------------
        # YOLO model (path resolve)
        # -------------------------------
        resolved, tried = _resolve_model_path(self._model_path_raw, package_name="cobot2")
        self._model_path = resolved

        if not os.path.isfile(self._model_path):
            self.get_logger().error(f"[YOLO_CAMERA] model not found: '{self._model_path_raw}'")
            self.get_logger().error("[YOLO_CAMERA] tried paths:")
            for p in tried:
                self.get_logger().error(f"  - {p}")
            raise FileNotFoundError(f"model file not found: {self._model_path_raw}")

        # 모폴로지 연산용 커널 (노이즈 제거용 5x5)
        self._morph_kernel = np.ones((5, 5), np.uint8)

        self.get_logger().info(f"[YOLO_CAMERA] loading model: {self._model_path}")
        self._yolo = YOLO(self._model_path)

        try:
            self._id_to_name: Dict[int, str] = dict(self._yolo.names)  # type: ignore[arg-type]
        except Exception:
            self._id_to_name = {}

        # -------------------------------
        # ROS pubs/subs
        # -------------------------------
        self._pub_err = self.create_publisher(Float32MultiArray, self._publish_topic, 10)

        # lock_done latched-like QoS (late-joiner safe)
        self._qos_lock_done = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_lock_done = self.create_publisher(Bool, self._lock_done_topic, self._qos_lock_done)

        # UI outputs
        self._pub_annotated = (
            self.create_publisher(Image, self._annotated_topic, 10) if self._publish_annotated else None
        )
        self._pub_ui_event = self.create_publisher(String, self._ui_event_topic, 10)
        self._ui_prev_has_target: bool = False
        self._ui_prev_locked_id: Optional[int] = None

        # follow enable gate subscriber
        self.create_subscription(Bool, self._follow_enable_topic, self._on_follow_enable, 10)

        # image subscription
        self._sub_image = None
        self._switch_image_topic(self._image_topic, reason="init")
        self.create_subscription(String, self._override_topic, self._on_override_topic, 10)

        # time-based switching
        self._is_day: Optional[bool] = None
        if self._enable_time_switch:
            self._timer = self.create_timer(self._time_check_period, self._on_time_check)

        # Lock-on state
        self._locked_id: Optional[int] = None
        self._locked_last_seen_t: float = 0.0
        self._lock_acquired_t: float = 0.0
        self._lock_done_published: bool = False

        # Last error for lock-done gating
        self._last_ex: Optional[float] = None
        self._last_ey: Optional[float] = None

        # Aim EMA state (px)
        self._aim_ema_x: Optional[float] = None
        self._aim_ema_y: Optional[float] = None

        # Debug / FPS
        self._t_prev = time.time()
        self._fps_ema = 0.0

        # debug cache
        self._dbg_last_aim_px: Optional[Tuple[int, int]] = None
        self._dbg_last_centroid_px: Optional[Tuple[int, int]] = None
        self._dbg_last_mode: str = "-"

        self.get_logger().info(
            f"[YOLO_CAMERA] ready (sub={self._image_topic}, model={self._model_path_raw} -> {self._model_path}, "
            f"target={self._target_class}, lock>={self._lock_conf_high}, keep>={self._maintain_conf_low}, "
            f"use_mask_centroid={self._use_mask_centroid}, aim_ema={self._aim_ema_enable} alpha={self._aim_ema_alpha:.2f}, "
            f"draw_bbox={self._debug_draw_bbox}, draw_mask={self._debug_draw_mask}, "
            f"follow_enable_default={self._follow_enable})"
        )

    # -------------------------------
    # Follow enable gate
    # -------------------------------
    def _on_follow_enable(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled == self._follow_enable:
            return

        self._follow_enable = enabled

        if not enabled:
            # 팔로우 OFF: 락/에러 상태 리셋
            self._reset_lock_state()

            # TRANSIENT_LOCAL 잔류 True 제거용으로 False 1회 publish
            try:
                self._pub_lock_done.publish(Bool(data=False))
            except Exception:
                pass

            # UI 상태 캐시도 강제로 "없음"으로 맞춰서 이벤트 스팸 방지
            self._ui_prev_has_target = False
            self._ui_prev_locked_id = None

            self._publish_ui_event("팔로우 비활성")
            self.get_logger().warn("[YOLO_CAMERA] follow_enable -> False (inference gated)")
        else:
            # 팔로우 ON: 찌꺼기 방지용 리셋 후 재개
            self._reset_lock_state()
            self._ui_prev_has_target = False
            self._ui_prev_locked_id = None

            self._publish_ui_event("팔로우 활성")
            self.get_logger().warn("[YOLO_CAMERA] follow_enable -> True (inference resumed)")

    # -------------------------------
    # Topic switching
    # -------------------------------
    def _reset_lock_state(self) -> None:
        self._locked_id = None
        self._locked_last_seen_t = 0.0
        self._lock_acquired_t = 0.0
        self._lock_done_published = False
        self._dbg_last_aim_px = None
        self._dbg_last_centroid_px = None
        self._dbg_last_mode = "-"
        self._aim_ema_x = None
        self._aim_ema_y = None
        self._last_ex = None
        self._last_ey = None

    def _switch_image_topic(self, new_topic: str, *, reason: str) -> None:
        new_topic = str(new_topic).strip()
        if not new_topic:
            return
        if new_topic == self._image_topic and self._sub_image is not None:
            return

        if self._sub_image is not None:
            try:
                self.destroy_subscription(self._sub_image)
            except Exception:
                pass
            self._sub_image = None

        self._image_topic = new_topic
        self._sub_image = self.create_subscription(Image, self._image_topic, self._on_image, 10)

        self._reset_lock_state()
        self.get_logger().warn(f"[YOLO_CAMERA] image_topic switched -> {self._image_topic} ({reason})")

    def _on_override_topic(self, msg: String) -> None:
        topic = str(msg.data).strip()
        if not topic:
            return
        self._switch_image_topic(topic, reason="override")

    def _is_daytime(self) -> bool:
        now = datetime.datetime.now().time()
        day_start = datetime.time(*[int(x) for x in self._day_hms[:3]])
        night_start = datetime.time(*[int(x) for x in self._night_hms[:3]])
        if day_start < night_start:
            return day_start <= now < night_start
        return now >= day_start or now < night_start

    def _on_time_check(self) -> None:
        is_day = self._is_daytime()
        if self._is_day is None:
            self._is_day = is_day
            target_topic = self._day_topic if is_day else self._night_topic
            self._switch_image_topic(target_topic, reason="time_init")
            return

        if is_day != self._is_day:
            self._is_day = is_day
            target_topic = self._day_topic if is_day else self._night_topic
            self._switch_image_topic(target_topic, reason="time_change")

    # -------------------------------
    # Publish
    # -------------------------------
    def _publish_error(self, ex: float, ey: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(ex), float(ey)]
        self._pub_err.publish(msg)

    def _publish_annotated_image(self, bgr: np.ndarray) -> None:
        if self._pub_annotated is None:
            return
        try:
            msg = self._bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self._pub_annotated.publish(msg)
        except Exception:
            return

    def _publish_ui_event(self, text: str) -> None:
        try:
            self._pub_ui_event.publish(String(data=str(text)))
        except Exception:
            return

    def _ui_update_transitions(self, has_target: bool) -> None:
        if has_target and (not self._ui_prev_has_target):
            self._publish_ui_event("탐지")
        if (not has_target) and self._ui_prev_has_target:
            self._publish_ui_event("탐지 해제")
        self._ui_prev_has_target = has_target

        if (self._locked_id is not None) and (self._ui_prev_locked_id is None):
            self._publish_ui_event("락온")
        if (self._locked_id is None) and (self._ui_prev_locked_id is not None):
            self._publish_ui_event("락온 해제")
        self._ui_prev_locked_id = self._locked_id

    def _maybe_publish_lock_done(self) -> None:
        if self._lock_done_published:
            return
        if self._locked_id is None or self._lock_acquired_t <= 0.0:
            return
        if self._last_ex is None or self._last_ey is None:
            return

        if abs(self._last_ex) > self._lock_done_err_th:
            return
        if abs(self._last_ey) > self._lock_done_err_th:
            return

        held = time.time() - self._lock_acquired_t
        if held >= self._lock_done_delay_sec:
            self._pub_lock_done.publish(Bool(data=True))
            self._publish_ui_event("락온 완료")
            self._lock_done_published = True

            self.get_logger().warn(
                f"[LOCK_DONE] id={self._locked_id} "
                f"held={held:.2f}s "
                f"err=({self._last_ex:.2f},{self._last_ey:.2f}) "
                f"topic={self._image_topic}"
            )

    # -------------------------------
    # YOLO inference + parse
    # -------------------------------
    def _run_inference(self, frame_bgr: np.ndarray):
        if self._use_tracker:
            return self._yolo.track(
                source=frame_bgr,
                conf=self._maintain_conf_low,
                persist=True,
                tracker=self._tracker_yaml,
                verbose=False,
                retina_masks=self._retina_masks,
            )
        return self._yolo.predict(
            source=frame_bgr,
            imgsz=self._imgsz,
            conf=self._maintain_conf_low,
            verbose=False,
            retina_masks=self._retina_masks,
        )

    def _refine_mask(self, mask_raw: np.ndarray, h: int, w: int) -> Optional[np.ndarray]:
        """
        [노이즈 제거] 마스크를 비트맵으로 그리고, 튀어나온 가시를 깎아낸 뒤(Open), 다시 윤곽선을 땀
        """
        mask_img = np.zeros((h, w), dtype=np.uint8)
        pts = mask_raw.astype(np.int32)
        cv2.fillPoly(mask_img, [pts], 255)

        mask_clean = cv2.morphologyEx(mask_img, cv2.MORPH_OPEN, self._morph_kernel)

        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < 50:
            return None

        return largest_contour.reshape(-1, 2)

    def _extract_detections(self, result, h: int, w: int) -> List[Detection]:
        dets: List[Detection] = []
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return dets

        masks = getattr(result, "masks", None)
        masks_xy = None
        if masks is not None and hasattr(masks, "xy"):
            masks_xy = masks.xy

        xyxy_np = boxes.xyxy.cpu().numpy()
        confs_np = boxes.conf.cpu().numpy()
        clss_np = boxes.cls.cpu().numpy()
        ids = getattr(boxes, "id", None)
        ids_np = ids.cpu().numpy().astype(int) if ids is not None else None

        for i, ((x1, y1, x2, y2), conf, cls_id) in enumerate(zip(xyxy_np, confs_np, clss_np)):
            cid = int(cls_id)
            cname = self._id_to_name.get(cid, str(cid))
            tid = int(ids_np[i]) if ids_np is not None else None

            contour_xy = None
            if masks_xy is not None:
                try:
                    raw_mask = np.asarray(masks_xy[i], dtype=np.float32)
                    if len(raw_mask) > 0:
                        contour_xy = self._refine_mask(raw_mask, h, w)
                except Exception:
                    contour_xy = None

            dets.append(
                Detection(
                    class_id=cid,
                    class_name=cname,
                    confidence=float(conf),
                    x1_px=float(x1),
                    y1_px=float(y1),
                    x2_px=float(x2),
                    y2_px=float(y2),
                    track_id=tid,
                    contour_xy=contour_xy,
                )
            )
        return dets

    def _pick_target_with_lock(self, dets: List[Detection]) -> Optional[Detection]:
        now = time.time()
        dets = [d for d in dets if d.class_name == self._target_class]

        if self._locked_id is not None:
            for d in dets:
                if d.track_id == self._locked_id:
                    self._locked_last_seen_t = now
                    return d
            if (now - self._locked_last_seen_t) > self._lost_timeout_sec:
                self._reset_lock_state()
            return None

        candidates = [d for d in dets if d.confidence >= self._lock_conf_high]
        if not candidates:
            return None

        target = max(candidates, key=lambda d: d.area)
        if target.track_id is not None:
            self._locked_id = target.track_id
            self._locked_last_seen_t = now
            self._lock_acquired_t = now
            self._lock_done_published = False
        return target

    # -------------------------------
    # Aim point (mask centroid -> fallback bbox) + EMA
    # -------------------------------
    @staticmethod
    def _ema(prev: float, cur: float, alpha: float) -> float:
        return (1.0 - alpha) * prev + alpha * cur

    def _apply_aim_ema(self, ax: float, ay: float) -> Tuple[float, float]:
        if not self._aim_ema_enable:
            return ax, ay
        a = float(np.clip(self._aim_ema_alpha, 0.0, 1.0))
        if self._aim_ema_x is None or self._aim_ema_y is None:
            self._aim_ema_x, self._aim_ema_y = ax, ay
        else:
            self._aim_ema_x = self._ema(self._aim_ema_x, ax, a)
            self._aim_ema_y = self._ema(self._aim_ema_y, ay, a)
        return self._aim_ema_x, self._aim_ema_y

    def _compute_aim_point(self, target: Detection) -> Tuple[Tuple[float, float], str]:
        if self._use_mask_centroid and target.contour_xy is not None and len(target.contour_xy) > 0:
            c = _centroid_from_contour(target.contour_xy)
            if c is not None:
                cx, cy, top_y = c
                height_span = max(0, cy - top_y)
                up = float(np.clip(self._mask_aim_up_ratio, 0.0, 1.0))
                aim_y = int(cy - (height_span * up))
                aim_x = int(cx)

                self._dbg_last_centroid_px = (cx, cy)
                self._dbg_last_aim_px = (aim_x, aim_y)
                return (float(aim_x), float(aim_y)), "mask"

        bx, by = target.bbox_center_px
        self._dbg_last_centroid_px = None
        self._dbg_last_aim_px = (int(bx), int(by))
        return (float(bx), float(by)), "bbox"

    # -------------------------------
    # Image callback
    # -------------------------------
    def _on_image(self, msg: Image) -> None:
        frame_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self._input_flip_v and self._input_flip_h:
            frame_bgr = cv2.flip(frame_bgr, -1)
        elif self._input_flip_v:
            frame_bgr = cv2.flip(frame_bgr, 0)
        elif self._input_flip_h:
            frame_bgr = cv2.flip(frame_bgr, 1)

        h, w = frame_bgr.shape[:2]

        # ✅ follow disabled: inference/lock/error input 차단
        if not self._follow_enable:
            self._last_ex = None
            self._last_ey = None
            self._publish_error(0.0, 0.0)

            if self._pub_annotated is not None:
                ann = frame_bgr.copy()
                cv2.putText(
                    ann,
                    "FOLLOW DISABLED",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                )
                self._publish_annotated_image(ann)

            if self._show_debug:
                cv2.imshow("yolo_camera_node", frame_bgr)
                cv2.waitKey(1)
            return

        # -------------------------------
        # normal mode
        # -------------------------------
        results = self._run_inference(frame_bgr)
        result0 = results[0]

        dets_all = self._extract_detections(result0, h, w)
        target = self._pick_target_with_lock(dets_all)

        if target is not None:
            (ax, ay), mode = self._compute_aim_point(target)
            ax, ay = self._apply_aim_ema(ax, ay)
            self._dbg_last_mode = f"{mode}+ema" if self._aim_ema_enable else mode

            ex, ey = _compute_error_norm((ax, ay), w, h)
            self._publish_error(ex, ey)
            self._last_ex = ex
            self._last_ey = ey
        else:
            self._dbg_last_aim_px = None
            self._dbg_last_centroid_px = None
            self._dbg_last_mode = "-"
            self._aim_ema_x = None
            self._aim_ema_y = None
            self._last_ex = None
            self._last_ey = None
            self._publish_error(0.0, 0.0)

        self._maybe_publish_lock_done()
        self._ui_update_transitions(target is not None)

        if self._pub_annotated is not None:
            ann = frame_bgr.copy()
            self._render_overlays(ann, target, w, h)
            self._publish_annotated_image(ann)

        if self._show_debug:
            self._draw_debug(frame_bgr, target, w, h)

    # -------------------------------
    # Debug draw
    # -------------------------------
    def _render_overlays(self, frame: np.ndarray, target: Optional[Detection], w: int, h: int) -> None:
        t = time.time()
        dt = max(1e-6, t - self._t_prev)
        self._t_prev = t
        fps = 1.0 / dt
        self._fps_ema = 0.9 * self._fps_ema + 0.1 * fps if self._fps_ema > 0.0 else fps

        cv2.circle(frame, (w // 2, h // 2), 6, (0, 255, 255), -1)
        cv2.putText(
            frame, f"FPS: {self._fps_ema:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2
        )

        lock_txt = f"LOCK: {self._locked_id}" if self._locked_id is not None else "LOCK: -"
        cv2.putText(frame, lock_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        done_txt = "DONE:1" if self._lock_done_published else "DONE:0"
        cv2.putText(frame, done_txt, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        if target is not None:
            if self._debug_draw_bbox:
                x1, y1, x2, y2 = map(int, [target.x1_px, target.y1_px, target.x2_px, target.y2_px])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if self._debug_draw_mask and self._mask_draw_contour and target.contour_xy is not None and len(target.contour_xy) > 0:
                cnt = target.contour_xy.astype(np.int32)
                cv2.polylines(frame, [cnt], True, (0, 255, 0), 2)

            if self._dbg_last_centroid_px is not None:
                cx, cy = self._dbg_last_centroid_px
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

            if self._dbg_last_aim_px is not None:
                if self._aim_ema_enable and (self._aim_ema_x is not None) and (self._aim_ema_y is not None):
                    ax, ay = int(self._aim_ema_x), int(self._aim_ema_y)
                else:
                    ax, ay = self._dbg_last_aim_px
                cv2.circle(frame, (ax, ay), 5, (0, 0, 255), -1)

            tid = target.track_id if target.track_id is not None else -1
            cv2.putText(
                frame,
                f"{target.class_name} id={tid} conf={target.confidence:.2f} mode={self._dbg_last_mode}",
                (10, 125),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

    def _draw_debug(self, frame: np.ndarray, target: Optional[Detection], w: int, h: int) -> None:
        self._render_overlays(frame, target, w, h)
        cv2.imshow("yolo_camera_node", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloCameraNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
