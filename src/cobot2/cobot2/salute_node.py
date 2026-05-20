#!/usr/bin/env python3
import time
import threading
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from dsr_msgs2.srv import MoveJoint

from cobot2.speak import speak

# 로컬 STT(Whisper) 사용: cobot2/STT.py 의 STT 클래스를 import
try:
    from cobot2.stt import STT
except Exception:
    STT = None


class SaluteRunner(Node):
    def __init__(self):
        super().__init__("salute_node")

        # ---- params ----
        self.declare_parameter("trigger_topic", "/salute_trigger")
        self.declare_parameter("done_topic", "/salute_done")
        self.declare_parameter("vel", 30.0)
        self.declare_parameter("acc", 50.0)
        self.declare_parameter("cooldown_sec", 2.0)
        self.declare_parameter("robot_ns", "/dsr01")
        self.declare_parameter("service_timeout_sec", 20.0)

        # ===== STT =====
        # 1) local STT(마이크 녹음+Whisper) 사용 여부
        self.declare_parameter("use_local_stt", True)

        # 2) 외부 STT 노드(토픽 기반) 사용 시: 트리거/텍스트 토픽
        self.declare_parameter("stt_trigger_topic", "/stt_trigger")
        self.declare_parameter("stt_topic", "/stt_text")         # String 토픽
        self.declare_parameter("listen_sec", 7.0)               # 기본 7초 듣기

        # 3) 들은 결과를 "스트링으로 내보내는" 토픽 (String publish)
        self.declare_parameter("stt_output_topic", "/salute_heard_text")
        self.declare_parameter("accord_output_topic", "/salute_accord_topic")
        # ===== TTS =====
        self.declare_parameter("tts_enabled", True)
        self.declare_parameter("tts_text", "충성! 수고하십니다!")   # J_SALUTE2 때 말할 문장
        self.declare_parameter("pre_tts_text", "누구십니까?")
        self.declare_parameter("post_tts_text", "고생하십쇼!")
        self.declare_parameter("post_tts_delay_sec", 3.0)

        # ---- load params ----
        self.trigger_topic = str(self.get_parameter("trigger_topic").value)
        self.done_topic = str(self.get_parameter("done_topic").value)
        self.vel = float(self.get_parameter("vel").value)
        self.acc = float(self.get_parameter("acc").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)

        self.robot_ns = str(self.get_parameter("robot_ns").value).rstrip("/")
        if self.robot_ns == "":
            self.robot_ns = "/dsr01"
        self.service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)

        self.use_local_stt = bool(self.get_parameter("use_local_stt").value)
        self.stt_trigger_topic = str(self.get_parameter("stt_trigger_topic").value)
        self.stt_topic = str(self.get_parameter("stt_topic").value)
        self.listen_sec = float(self.get_parameter("listen_sec").value)
        self.stt_output_topic = str(self.get_parameter("stt_output_topic").value)
        self.accord_output_topic = str(self.get_parameter("accord_output_topic").value)

        self.tts_enabled = bool(self.get_parameter("tts_enabled").value)
        self.tts_text = str(self.get_parameter("tts_text").value)
        self.pre_tts_text = str(self.get_parameter("pre_tts_text").value)
        self.post_tts_text = str(self.get_parameter("post_tts_text").value)
        self.post_tts_delay_sec = float(self.get_parameter("post_tts_delay_sec").value)

        # ---- pub/sub ----
        self.sub = self.create_subscription(Bool, self.trigger_topic, self._on_trigger, 10)
        self.pub_done = self.create_publisher(Bool, self.done_topic, 10)

        # "들은 텍스트" 출력 토픽
        self.pub_heard_text = self.create_publisher(String, self.stt_output_topic, 10)
        self.pub_accord_text = self.create_publisher(String, self.accord_output_topic, 10)

        # 외부 STT(토픽 기반)용 pub/sub
        self.sub_stt = self.create_subscription(String, self.stt_topic, self._on_stt, 10)
        self.pub_stt_trigger = self.create_publisher(Bool, self.stt_trigger_topic, 10)

        # ---- state ----
        self._busy = False
        self._lock = threading.Lock()
        self._last_time = 0.0

        # 외부 STT listen state
        self._stt_lock = threading.Lock()
        self._listening = False
        self._heard_text = ""
        self._heard_event = threading.Event()

        # ---- service client ----
        self._srv_move_joint_name = f"{self.robot_ns}/motion/move_joint"
        self._cli_move_joint = self.create_client(MoveJoint, self._srv_move_joint_name)

        self.get_logger().info(
            f"Subscribed: {self.trigger_topic} (Bool). True -> salute\n"
            f"STT mode: {'LOCAL(STT.py)' if self.use_local_stt else 'TOPIC(/stt_text)'} listen={self.listen_sec}s\n"
            f"STT input: {self.stt_topic} (String), trigger: {self.stt_trigger_topic} (Bool)\n"
            f"STT output: {self.stt_output_topic} (String)\n"
            f"Publish done: {self.done_topic} (Bool)\n"
            f"Using service: {self._srv_move_joint_name}\n"
            f"TTS: {'ON' if self.tts_enabled else 'OFF'} / mid=\"{self.tts_text}\""
        )

        if self.use_local_stt and STT is None:
            self.get_logger().warn(
                "use_local_stt=True 이지만 cobot2.STT import 실패. "
                "패키지 경로/파일명(cobot2/STT.py) 확인하세요. "
                "임시로 토픽 기반 STT로 fallback 합니다."
            )
            self.use_local_stt = False

    def _on_trigger(self, msg: Bool):
        if not msg.data:
            return

        now = time.time()
        with self._lock:
            if self._busy:
                self.get_logger().warn("Ignored: already running.")
                return
            if now - self._last_time < self.cooldown_sec:
                self.get_logger().warn("Ignored: cooldown.")
                return
            self._busy = True
            self._last_time = now

        threading.Thread(target=self._run, daemon=True).start()

    # ===== 외부 STT 토픽 콜백 =====
    def _on_stt(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return
        with self._stt_lock:
            if not self._listening:
                return
            self._heard_text = text
            self._heard_event.set()

    def _listen_from_topic(self, timeout_sec: float) -> str:
        """/stt_text 토픽으로 들어오는 '마지막 문장' 기다림."""
        with self._stt_lock:
            self._listening = True
            self._heard_text = ""
        self._heard_event.clear()


        # 외부 STT 노드 트리거
        self.pub_stt_trigger.publish(Bool(data=True))

        got = self._heard_event.wait(timeout=timeout_sec)

        with self._stt_lock:
            self._listening = False
            return self._heard_text if got else ""

    # ===== 로컬 STT (cobot2/STT.py) =====
    def _listen_local_stt(self, duration_sec: float) -> str:
        """STT.py를 직접 호출해서 마이크 녹음 후 문자열 반환."""
        if STT is None:
            return ""
        try:
            stt = STT(duration=float(duration_sec))
            text = stt.speech2text()
            return (text or "").strip()
        except Exception:
            self.get_logger().error("Local STT failed:\n" + traceback.format_exc())
            return ""

    def _say(self, text: str):
        if self.tts_enabled and text and text.strip():
            speak(text)

    def _run(self):
        ok = False
        try:
            self.pub_accord_text.publish(String(data="암구호 일치, 신원 정보 확인 시작"))
            self.get_logger().info("Published accord: 암구호 일치")

            # 1) 경례 전에 '누구십니까?'
            self.get_logger().info("TTS : 누구십니까?")
            self._say(self.pre_tts_text)
            time.sleep(0.2)

            # 2) 듣기 + 결과를 문자열로 publish + 로그 출력
            listen_start = time.time()  # 7초(또는 listen_sec) 동안은 반드시 대기
            if self.use_local_stt:
                self.get_logger().info(f"Listening (LOCAL STT) for {self.listen_sec:.1f}s ...")
                heard = self._listen_local_stt(self.listen_sec)
            else:
                self.get_logger().info(f"Listening (TOPIC STT) for {self.listen_sec:.1f}s ...")
                heard = self._listen_from_topic(self.listen_sec)

            # STT가 빨리 반환해도, 총 listen_sec 만큼은 기다린 뒤 동작 시작
            elapsed = time.time() - listen_start
            remain = self.listen_sec - elapsed
            if remain > 0:
                time.sleep(remain)

            if heard:
                self.get_logger().info(f"Heard answer: {heard!r}")
                self.pub_heard_text.publish(String(data=heard))  # 스트링으로 내보내기
            else:
                self.get_logger().warn("Heard answer: (none)")
                self.pub_heard_text.publish(String(data=""))  # 빈 문자열도 publish (원하면 제거 가능)

            # 3) 기존 경례 동작 수행
            ok = self._salute_motion()

            # 4) 동작 완전히 끝난 후 3초 뒤 '고생하십쇼!'
            if ok:
                time.sleep(self.post_tts_delay_sec)
                self.get_logger().info("TTS : 고생하십쇼!")
                self._say(self.post_tts_text)

        except Exception:
            self.get_logger().error("Salute failed:\n" + traceback.format_exc())
            ok = False
        finally:
            self.pub_done.publish(Bool(data=ok))
            with self._lock:
                self._busy = False

    def _call_movej(self, pos, vel, acc, t=0.0) -> bool:
        req = MoveJoint.Request()
        req.pos = [float(x) for x in pos]
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = float(t)
        req.radius = 0.0
        req.mode = 0  # 기본 모드

        # 필드가 있을 때만 세팅 (환경별 msg 생성 차이 대비)
        if hasattr(req, "blend_type"):
            req.blend_type = 0
        if hasattr(req, "sync_type"):
            req.sync_type = 0
        self.get_logger().info(f"MoveJoint fields: {req.__slots__}")

        future = self._cli_move_joint.call_async(req)

        start = time.time()
        while rclpy.ok() and not future.done():
            if (time.time() - start) > self.service_timeout_sec:
                raise TimeoutError(f"move_joint timeout ({self.service_timeout_sec}s)")
            time.sleep(0.01)

        resp = future.result()
        if resp is None:
            raise RuntimeError("move_joint got no response")

        if not resp.success:
            self.get_logger().error("move_joint returned success=False")
        return bool(resp.success)

    def _salute_motion(self) -> bool:
        vel = self.vel
        acc = self.acc

        J_READY = [-4.2, -24.8, -122.5, 175.1, -57.4, 92.7]
        J_SALUTE1 = [-90, -80, 0, 90, 0, 90]
        J_SALUTE2 = [-90, -90, 125, 90, 0, 90]

        # self.get_logger().info("move to READY")
        # if not self._call_movej(J_READY, vel=vel, acc=acc):
        #     return False

        self.get_logger().info("SALUTE START")
        if not self._call_movej(J_SALUTE1, vel=vel, acc=acc):
            return False
        time.sleep(1.0)

        # J_SALUTE2 동작할 때 '충성! 수고하십니다!'
        if self.tts_enabled and self.tts_text.strip():
            self.get_logger().info("TTS : 충성! 수고하십니다!")
            self._say(self.tts_text)

        if not self._call_movej(J_SALUTE2, vel=50, acc=acc):
            return False
        time.sleep(2.0)

        self.get_logger().info("back to READY")
        if not self._call_movej(J_READY, vel=vel, acc=acc):
            return False

        return True


def main():
    rclpy.init()
    node = SaluteRunner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()