# tcp_follow_node v1.120 2026-02-02
# [이번 버전에서 수정된 사항]
# - (기능구현) J4 하한 리미트 추가: J4 <= limit_j4_min_deg면 추종 명령(speedl)을 0으로 컷(hold), J4가 (min+release_margin) 이상 회복 시 자동 재개
# - (유지) startup movej(main()에서 executor.spin 이전 1회 실행), settle/필터리셋, Y/Z speedl 추종 및 base Y/Z 절대 리미트 유지
# - (유지) B(ry) 회전 추종(enable_b_rotation) 및 반응성 튜닝 기본값 유지

from __future__ import annotations

import time
import threading
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

import DR_init
from dataclasses import dataclass
# ==========================================================
# ROBOT constants
# ==========================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot(node: Node):
    """main()에서 노드 준비 후 1회만 호출."""
    import DSR_ROBOT2 as dr

    try:
        dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
    except Exception:
        pass

    dr.set_tool(ROBOT_TOOL)
    dr.set_tcp(ROBOT_TCP)
    return dr
# ==========================================================
# Follow params
# ==========================================================
@dataclass
class FollowParamsYZ:
    vy_mm_s_per_error: float
    vz_mm_s_per_error: float
    vmax_y_mm_s: float
    vmax_z_mm_s: float
    deadzone_error_norm: float
    filter_alpha: float
    y_sign: float
    z_sign: float


class RobotInterface:
    def __init__(self, node: Node, *, dry_run: bool):
        self._node = node
        self._dry_run = dry_run
        self._dr = None

    def set_dr(self, dr) -> None:
        self._dr = dr
        if not hasattr(self._dr, "speedl"):
            raise AttributeError("DSR_ROBOT2 missing speedl()")

    def movej_startup(self, joints_deg: List[float], *, vel: float, acc: float) -> None:
        if self._dry_run or self._dr is None:
            return
        self._dr.movej(joints_deg, vel=vel, acc=acc)

    def speedl(
        self,
        vel_6: Tuple[float, float, float, float, float, float],
        *,
        acc: float,
        time_s: float,
    ) -> None:
        if self._dry_run or self._dr is None:
            return
            
        try:
            self._dr.speedl(list(vel_6), acc, time_s)
            return
        except TypeError:
            pass

    def get_current_posx(self):
        if self._dry_run or self._dr is None:
            return None
        if not hasattr(self._dr, "get_current_posx"):
            return None
        try:
            out = self._dr.get_current_posx()
            if isinstance(out, (list, tuple)) and len(out) > 0:
                if isinstance(out[0], (list, tuple)) and len(out[0]) >= 6:
                    return list(out[0])[:6]
                if len(out) >= 6 and isinstance(out[0], (int, float)):
                    return list(out)[:6]
            return None
        except Exception:
            return None

    def get_current_posj(self):
        """Return current joint angles [j1..j6] deg if available, else None."""
        if self._dry_run or self._dr is None:
            return None
        if not hasattr(self._dr, "get_current_posj"):
            return None
        try:
            out = self._dr.get_current_posj()
            if isinstance(out, (list, tuple)) and len(out) > 0:
                if isinstance(out[0], (list, tuple)) and len(out[0]) >= 6:
                    return [float(x) for x in out[0][:6]]
                if len(out) >= 6 and isinstance(out[0], (int, float)):
                    return [float(x) for x in out[:6]]
            return None
        except Exception:
            return None


class TcpFollowNode(Node):
    def __init__(self) -> None:
        super().__init__("tcp_follow_node", namespace=ROBOT_ID)

        # ---- Params
        self.declare_parameter("dry_run", False)

        self.declare_parameter("startup_movej_enable", True)
        self.declare_parameter("startup_movej_joints_deg", [-4.0, -24.82, -122.52, 175.12, -57.42, 90.0])
        self.declare_parameter("startup_movej_vel", 60.0)
        self.declare_parameter("startup_movej_acc", 60.0)
        self.declare_parameter("startup_settle_sec", 0.8)

        self.declare_parameter("command_rate_hz", 40.0)
        self.declare_parameter("target_lost_timeout_sec", 0.5)
        self.declare_parameter("speedl_acc", 300.0)
        self.declare_parameter("speedl_time_scale", 1.2)

        # ---- responsiveness tuned defaults
        self.declare_parameter("vy_mm_s_per_error", 600.0)
        self.declare_parameter("vz_mm_s_per_error", 600.0)
        self.declare_parameter("vmax_y_mm_s", 600.0)
        self.declare_parameter("vmax_z_mm_s", 600.0)

        # ✅ NEW: minimum velocity (vmin)
        self.declare_parameter("vmin_y_mm_s", 25.0)
        self.declare_parameter("vmin_z_mm_s", 25.0)

        self.declare_parameter("deadzone_error_norm", 0.015)
        self.declare_parameter("filter_alpha", 0.6)
        self.declare_parameter("y_sign", -1.0)
        self.declare_parameter("z_sign", -1.0)

        self.declare_parameter("error_topic", "/follow/error_norm")

        # ---- tcp_follow enable switch
        self.declare_parameter("enable_topic", "/follow/enable")

        # ---- base Y/Z absolute limits
        self.declare_parameter("limit_base_y_enable", True)
        self.declare_parameter("limit_base_y_min_mm", -500.0)
        self.declare_parameter("limit_base_y_max_mm", 500.0)

        self.declare_parameter("limit_base_z_enable", True)
        self.declare_parameter("limit_base_z_min_mm", 200.0)
        self.declare_parameter("limit_base_z_max_mm", 600.0)

        self.declare_parameter("limit_base_yz_poll_hz", 60.0)

        # ---- B(=posx ry) rotation tracking
        self.declare_parameter("enable_b_rotation", False)
        self.declare_parameter("wb_deg_s_per_error", 12.0)
        self.declare_parameter("wmax_b_deg_s", 18.0)
        # ✅ NEW
        self.declare_parameter("vmin_b_deg_s", 5.0)
        self.declare_parameter("b_sign", 1.0)

        # ---- (NEW) J4 limit
        self.declare_parameter("limit_j4_enable", True)
        self.declare_parameter("limit_j4_min_deg", -80.0)
        self.declare_parameter("limit_j4_release_margin_deg", 2.0)  # -78도 이상에서 해제(기본)
        self.declare_parameter("limit_j4_poll_hz", 20.0)

        # posx monitor
        self.declare_parameter("debug_posx_enable", True)
        self.declare_parameter("debug_posx_rate_hz", 60.0)
        self.declare_parameter("debug_posx_pub", True)
        self.declare_parameter("debug_posx_topic", "/follow/posx_debug")
        self.declare_parameter("debug_dposx_topic", "/follow/dposx_debug")

        # ---- Read params
        self._dry_run: bool = bool(self.get_parameter("dry_run").value)

        self._startup_movej_enable: bool = bool(self.get_parameter("startup_movej_enable").value)
        self._startup_movej_joints_deg: List[float] = list(self.get_parameter("startup_movej_joints_deg").value)
        self._startup_movej_vel: float = float(self.get_parameter("startup_movej_vel").value)
        self._startup_movej_acc: float = float(self.get_parameter("startup_movej_acc").value)
        self._startup_settle_sec: float = float(self.get_parameter("startup_settle_sec").value)

        self._command_rate_hz: float = float(self.get_parameter("command_rate_hz").value)
        self._target_lost_timeout_sec: float = float(self.get_parameter("target_lost_timeout_sec").value)
        self._speedl_acc: float = float(self.get_parameter("speedl_acc").value)
        self._speedl_time_scale: float = float(self.get_parameter("speedl_time_scale").value)

        self._vmin_y_mm_s: float = float(self.get_parameter("vmin_y_mm_s").value)
        self._vmin_z_mm_s: float = float(self.get_parameter("vmin_z_mm_s").value)
        self._vmin_b_deg_s: float = float(self.get_parameter("vmin_b_deg_s").value)

        self._params = FollowParamsYZ(
            vy_mm_s_per_error=float(self.get_parameter("vy_mm_s_per_error").value),
            vz_mm_s_per_error=float(self.get_parameter("vz_mm_s_per_error").value),
            vmax_y_mm_s=float(self.get_parameter("vmax_y_mm_s").value),
            vmax_z_mm_s=float(self.get_parameter("vmax_z_mm_s").value),
            deadzone_error_norm=float(self.get_parameter("deadzone_error_norm").value),
            filter_alpha=float(self.get_parameter("filter_alpha").value),
            y_sign=float(self.get_parameter("y_sign").value),
            z_sign=float(self.get_parameter("z_sign").value),
        )

        self._error_topic: str = str(self.get_parameter("error_topic").value)

        # tcp follow switch variable
        self._enable_topic: str = str(self.get_parameter("enable_topic").value)

        # limits (base y/z)
        self._limit_y_enable: bool = bool(self.get_parameter("limit_base_y_enable").value)
        self._limit_y_min: float = float(self.get_parameter("limit_base_y_min_mm").value)
        self._limit_y_max: float = float(self.get_parameter("limit_base_y_max_mm").value)

        self._limit_z_enable: bool = bool(self.get_parameter("limit_base_z_enable").value)
        self._limit_z_min: float = float(self.get_parameter("limit_base_z_min_mm").value)
        self._limit_z_max: float = float(self.get_parameter("limit_base_z_max_mm").value)

        self._limit_poll_hz: float = float(self.get_parameter("limit_base_yz_poll_hz").value)

        # B rotation tracking
        self._enable_b_rotation: bool = bool(self.get_parameter("enable_b_rotation").value)
        self._wb_deg_s_per_error: float = float(self.get_parameter("wb_deg_s_per_error").value)
        self._wmax_b_deg_s: float = float(self.get_parameter("wmax_b_deg_s").value)
        self._b_sign: float = float(self.get_parameter("b_sign").value)

        # J4 limit
        self._limit_j4_enable: bool = bool(self.get_parameter("limit_j4_enable").value)
        self._limit_j4_min_deg: float = float(self.get_parameter("limit_j4_min_deg").value)
        self._limit_j4_release_margin_deg: float = float(self.get_parameter("limit_j4_release_margin_deg").value)
        self._limit_j4_poll_hz: float = float(self.get_parameter("limit_j4_poll_hz").value)

        self._dbg_posx_enable: bool = bool(self.get_parameter("debug_posx_enable").value)
        self._dbg_posx_rate_hz: float = float(self.get_parameter("debug_posx_rate_hz").value)
        self._dbg_posx_pub: bool = bool(self.get_parameter("debug_posx_pub").value)
        self._dbg_posx_topic: str = str(self.get_parameter("debug_posx_topic").value)
        self._dbg_dposx_topic: str = str(self.get_parameter("debug_dposx_topic").value)

        self._robot = RobotInterface(self, dry_run=self._dry_run)

        self._latest_error_norm: Optional[Tuple[float, float]] = None
        self._latest_error_time_sec: float = 0.0
        self._err_lock = threading.Lock()

        # enable runtime state / Lock could be resolve threading issue
        self._enabled: bool = True
        self._en_lock = threading.Lock()

        self._filt_ex: float = 0.0
        self._filt_ey: float = 0.0
        self._have_filter: bool = False

        self._startup_done: bool = False

        self._posx_prev: Optional[List[float]] = None
        self._posx_prev_t: Optional[float] = None

        # limit runtime state (base y/z)
        self._lim_lock = threading.Lock()
        self._y_latest: Optional[float] = None
        self._z_latest: Optional[float] = None
        self._last_poll_t: float = 0.0
        self._warn_t: float = 0.0

        # limit runtime state (J4)
        self._j_lock = threading.Lock()
        self._j4_latest: Optional[float] = None
        self._j_last_poll_t: float = 0.0
        self._j4_hold_active: bool = False
        self._j4_warn_t: float = 0.0

        self.create_subscription(Float32MultiArray, self._error_topic, self._on_error_norm, 10)

        # switch topic sub
        self.create_subscription(Bool, self._enable_topic, self._on_enable, 10)

        if self._dbg_posx_pub:
            self._pub_posx = self.create_publisher(Float32MultiArray, self._dbg_posx_topic, 10)
            self._pub_dposx = self.create_publisher(Float32MultiArray, self._dbg_dposx_topic, 10)
        else:
            self._pub_posx = None
            self._pub_dposx = None

        # sanity
        if self._limit_y_enable and (self._limit_y_min >= self._limit_y_max):
            self._limit_y_enable = False
        if self._limit_z_enable and (self._limit_z_min >= self._limit_z_max):
            self._limit_z_enable = False

    def set_dr(self, dr) -> None:
        self._robot.set_dr(dr)

    def destroy_node(self):
        super().destroy_node()

    def _on_error_norm(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        ex, ey = float(msg.data[0]), float(msg.data[1])
        with self._err_lock:
            self._latest_error_norm = (ex, ey)
            self._latest_error_time_sec = time.time()
    
    # add 2 helper functions / tcp follow switch
    def _on_enable(self, msg: Bool) -> None:
        # 사실상 temp 처럼 임시
        new_enabled = bool(msg.data)
        with self._en_lock:
            prev_enabled = self._enabled
            self._enabled = new_enabled
        
        # falling edge로
        if prev_enabled and (not new_enabled):
            if self._startup_done:
                dt = 1.0 / max(self._command_rate_hz, 1.0)
                cmd_time = dt * max(self._speedl_time_scale, 1.0)
                self._robot.speedl((0.,0.,0.,0.,0.,0.), acc=self._speedl_acc, time_s=cmd_time)

    def _is_enabled(self) -> bool:
        with self._en_lock:
            return self._enabled


    def _target_alive(self) -> bool:
        with self._err_lock:
            if self._latest_error_norm is None:
                return False
            return (time.time() - self._latest_error_time_sec) <= self._target_lost_timeout_sec

    def _get_latest_error(self) -> Optional[Tuple[float, float]]:
        with self._err_lock:
            return self._latest_error_norm

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return lo if v < lo else hi if v > hi else v
    
    @staticmethod
    def _apply_vmin(v: float, vmin: float) -> float:
        if abs(v) < 1e-6:
            return 0.0
        return v if abs(v) >= vmin else (vmin if v > 0.0 else -vmin)

    def _ema(self, prev: float, cur: float, alpha: float) -> float:
        return (1.0 - alpha) * prev + alpha * cur

    def finalize_startup_after_movej(self) -> None:
        if self._startup_done:
            return

        if self._startup_settle_sec > 0.0:
            time.sleep(self._startup_settle_sec)

        with self._err_lock:
            self._latest_error_norm = None
            self._latest_error_time_sec = 0.0
        self._have_filter = False
        self._filt_ex = 0.0
        self._filt_ey = 0.0

        # prime base y/z latest
        if self._limit_y_enable or self._limit_z_enable:
            posx = self._robot.get_current_posx()
            if posx is not None and len(posx) >= 3:
                with self._lim_lock:
                    self._y_latest = float(posx[1])
                    self._z_latest = float(posx[2])
                    self._last_poll_t = time.time()
        self._startup_done = True

    # -----------------------------
    # base y/z polling + limit
    # -----------------------------
    def _poll_base_yz_if_needed(self) -> None:
        if not (self._limit_y_enable or self._limit_z_enable):
            return
        if self._limit_poll_hz <= 0.0:
            return

        now = time.time()
        dt_need = 1.0 / max(self._limit_poll_hz, 1e-3)

        with self._lim_lock:
            last = self._last_poll_t
        if (now - last) < dt_need:
            return

        posx = self._robot.get_current_posx()
        if posx is None or len(posx) < 3:
            return

        y = float(posx[1])
        z = float(posx[2])
        with self._lim_lock:
            self._y_latest = y
            self._z_latest = z
            self._last_poll_t = now

    def _throttled_limit_warn(self, msg: str, period_sec: float = 1.0) -> None:
        now = time.time()
        with self._lim_lock:
            if (now - self._warn_t) < period_sec:
                return
            self._warn_t = now
        self.get_logger().warn(msg)

    def _apply_base_yz_limits(self, vy: float, vz: float) -> Tuple[float, float]:
        if not (self._limit_y_enable or self._limit_z_enable):
            return vy, vz

        with self._lim_lock:
            y = self._y_latest
            z = self._z_latest

        if self._limit_y_enable and (y is not None):
            if (y >= self._limit_y_max) and (vy > 0.0):
                self._throttled_limit_warn(f"base_y_limit hit: y={y:.1f} >= {self._limit_y_max:.1f} -> vy cut")
                vy = 0.0
            if (y <= self._limit_y_min) and (vy < 0.0):
                self._throttled_limit_warn(f"base_y_limit hit: y={y:.1f} <= {self._limit_y_min:.1f} -> vy cut")
                vy = 0.0

        if self._limit_z_enable and (z is not None):
            if (z >= self._limit_z_max) and (vz > 0.0):
                self._throttled_limit_warn(f"base_z_limit hit: z={z:.1f} >= {self._limit_z_max:.1f} -> vz cut")
                vz = 0.0
            if (z <= self._limit_z_min) and (vz < 0.0):
                self._throttled_limit_warn(f"base_z_limit hit: z={z:.1f} <= {self._limit_z_min:.1f} -> vz cut")
                vz = 0.0

        return vy, vz

    # -----------------------------
    # J4 polling + hold limit
    # -----------------------------
    def _poll_j4_if_needed(self) -> None:
        if not self._limit_j4_enable:
            return
        if self._limit_j4_poll_hz <= 0.0:
            return

        now = time.time()
        dt_need = 1.0 / max(self._limit_j4_poll_hz, 1e-3)

        with self._j_lock:
            last = self._j_last_poll_t
        if (now - last) < dt_need:
            return

        posj = self._robot.get_current_posj()
        if posj is None or len(posj) < 4:
            return

        j4 = float(posj[3])  # J4
        with self._j_lock:
            self._j4_latest = j4
            self._j_last_poll_t = now

    def _throttled_j4_warn(self, msg: str, period_sec: float = 1.0) -> None:
        now = time.time()
        with self._j_lock:
            if (now - self._j4_warn_t) < period_sec:
                return
            self._j4_warn_t = now
        self.get_logger().warn(msg)

    def _apply_j4_limit_hold(self, vy: float, vz: float, wy: float) -> Tuple[float, float, float]:
        if not self._limit_j4_enable:
            return vy, vz, wy

        with self._j_lock:
            j4 = self._j4_latest
            hold = self._j4_hold_active

        if j4 is None:
            return vy, vz, wy

        min_deg = float(self._limit_j4_min_deg)
        release_deg = min_deg + max(0.0, float(self._limit_j4_release_margin_deg))

        if hold:
            if j4 >= release_deg:
                with self._j_lock:
                    self._j4_hold_active = False
                return vy, vz, wy
            return 0.0, 0.0, 0.0

        if j4 <= min_deg:
            with self._j_lock:
                self._j4_hold_active = True
            self._throttled_j4_warn(f"J4 limit hit: j4={j4:.1f} <= {min_deg:.1f} deg -> hold (speedl cut)")
            return 0.0, 0.0, 0.0

        return vy, vz, wy

    # -----------------------------
    # posx monitor (POSE/DPOS)
    # -----------------------------
    def _posx_monitor_loop(self):
        dt = 1.0 / max(self._dbg_posx_rate_hz, 1.0)
        while rclpy.ok():
            if not self._dbg_posx_enable:
                time.sleep(dt)
                continue

            posx = self._robot.get_current_posx()
            now = time.time()

            if posx is not None:
                if self._pub_posx is not None:
                    m = Float32MultiArray()
                    m.data = [float(v) for v in posx]
                    self._pub_posx.publish(m)

                if self._posx_prev is not None and self._posx_prev_t is not None:
                    d = [posx[i] - self._posx_prev[i] for i in range(6)]
                    dt_sec = max(1e-6, now - self._posx_prev_t)

                    if self._pub_dposx is not None:
                        md = Float32MultiArray()
                        md.data = [float(v) for v in d]
                        self._pub_dposx.publish(md)
                self._posx_prev = posx
                self._posx_prev_t = now

            time.sleep(dt)

    # -----------------------------
    # speedl loop
    # -----------------------------
    def spin_speedl_loop(self) -> None:
        dt = 1.0 / max(self._command_rate_hz, 1.0)
        cmd_time = dt * max(self._speedl_time_scale, 1.0)

        if self._dbg_posx_enable:
            threading.Thread(target=self._posx_monitor_loop, daemon=True).start()

        while rclpy.ok():
            if not self._startup_done:
                # 혹시 dsr 이랑 tcp_follow_node 에서 다시 move 치대한 안불러 보자
                # self._robot.speedl((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), acc=self._speedl_acc, time_s=cmd_time)
                time.sleep(dt)
                continue
            
            # orchestrator enable switch
            if not self._is_enabled():
                # 혹시 dsr 이랑 tcp_follow_node 에서 다시 move 치대한 안불러 보자
                # self._robot.speedl((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), acc=self._speedl_acc, time_s=cmd_time)
                time.sleep(dt)
                continue

            self._poll_base_yz_if_needed()
            self._poll_j4_if_needed()

            if not self._target_alive():
                # 혹시 dsr 이랑 tcp_follow_node 에서 다시 move 치대한 안불러 보자
                # self._robot.speedl((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), acc=self._speedl_acc, time_s=cmd_time)
                time.sleep(dt)
                continue

            e = self._get_latest_error()
            if e is None:
                # 혹시 dsr 이랑 tcp_follow_node 에서 다시 move 치대한 안불러 보자 / 비상정지가 홈으로...
                # self._robot.speedl((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), acc=self._speedl_acc, time_s=cmd_time)
                time.sleep(dt)
                continue

            ex, ey = e

            if abs(ex) < self._params.deadzone_error_norm:
                ex = 0.0
            if abs(ey) < self._params.deadzone_error_norm:
                ey = 0.0

            if not self._have_filter:
                self._filt_ex, self._filt_ey = ex, ey
                self._have_filter = True
            else:
                self._filt_ex = self._ema(self._filt_ex, ex, self._params.filter_alpha)
                self._filt_ey = self._ema(self._filt_ey, ey, self._params.filter_alpha)

            fx, fy = self._filt_ex, self._filt_ey

            vy = self._params.y_sign * (self._params.vy_mm_s_per_error * fx)
            vz = self._params.z_sign * (self._params.vz_mm_s_per_error * fy)

            vy = self._clamp(vy, -self._params.vmax_y_mm_s, self._params.vmax_y_mm_s)
            vz = self._clamp(vz, -self._params.vmax_z_mm_s, self._params.vmax_z_mm_s)

            # NEWWWWWWWWWW
            vy = self._apply_vmin(vy, self._vmin_y_mm_s)
            vz = self._apply_vmin(vz, self._vmin_z_mm_s)

            wy = 0.0
            if self._enable_b_rotation:
                wy = self._b_sign * (self._wb_deg_s_per_error * fx)
                wy = self._clamp(wy, -self._wmax_b_deg_s, self._wmax_b_deg_s)
                # 아래 줄로 실행을 안해도 되나
                # wy = self._apply_vmin(wy, self._vmin_b_deg_s)

            # base Y/Z 절대 리미트 적용
            vy, vz = self._apply_base_yz_limits(vy, vz)

            # J4 리미트 적용 (걸리면 vy/vz/wy 모두 0으로 컷)
            vy, vz, wy = self._apply_j4_limit_hold(vy, vz, wy)

            self._robot.speedl((0.0, vy, vz, 0.0, wy, 0.0), acc=self._speedl_acc, time_s=cmd_time)
            time.sleep(dt)


def main(args=None) -> None:
    rclpy.init(args=args)

    follow_node = TcpFollowNode()

    dsr_node = rclpy.create_node("dsr_internal_worker", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    try:
        dr = initialize_robot(follow_node)
        follow_node.set_dr(dr)

        # startup movej는 executor.spin() 전에만 1회 호출
        if bool(follow_node.get_parameter("startup_movej_enable").value):
            joints = list(follow_node.get_parameter("startup_movej_joints_deg").value)
            vel = float(follow_node.get_parameter("startup_movej_vel").value)
            acc = float(follow_node.get_parameter("startup_movej_acc").value)
            follow_node._robot.movej_startup(joints, vel=vel, acc=acc)

        follow_node.finalize_startup_after_movej()

    except Exception as e:
        follow_node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(follow_node)
    executor.add_node(dsr_node)

    t = threading.Thread(target=follow_node.spin_speedl_loop, daemon=True)
    t.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            follow_node.destroy_node()
            dsr_node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()