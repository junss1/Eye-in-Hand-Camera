# orchestrator v0.711 2026-02-05
# [이번 버전에서 수정된 사항]
# - (유지) /follow/ui_event 로 단계 이벤트 publish 유지(시간은 로거에서 찍음)
# - (유지) 기능은 그대로 두고, ui_event 문구를 사람 친화적으로 변경
# - (유지) 날짜별 문어/답어 딕셔너리(WEEKLY_PHRASES) 자동 선택, lock_done IDLE gate, follow 타이밍( AUTH 중 ON / salute·shoot 직전 OFF / done 후 ON ) 유지

from __future__ import annotations

import datetime
import time
from zoneinfo import ZoneInfo

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, String

from cobot2_interfaces.action import Auth

from cobot2.speak import speak


# =========================
# 날짜별 문어/답어 딕셔너리
# - 키: "YYYY-MM-DD"
# - 값: ("문어(challenge)", "답어(expected)")
# =========================
WEEKLY_PHRASES: dict[str, tuple[str, str]] = {
    "2026-02-05": ("아이폰", "갤럭시"),
    "2026-02-06": ("빨강", "파랑"),
    "2026-02-07": ("고양이", "강아지"),
    "2026-02-08": ("아메리카노", "라떼"),
    "2026-02-09": ("사과", "바나나"),
    "2026-02-10": ("해", "달"),
    "2026-02-11": ("봄", "가을"),
}


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("orchestrator_node")
        self._cbg = ReentrantCallbackGroup()

        # start, status : for test
        self.declare_parameter("start_topic", "/orchestrator/start")
        self.declare_parameter("status_topic", "/orchestrator/status")  # orche 상태 출력용

        # for motion nodes (topic trigger)
        self.declare_parameter("salute_trigger_topic", "/salute_trigger")
        self.declare_parameter("salute_done_topic", "/salute_done")
        self.declare_parameter("shoot_trigger_topic", "/shoot_trigger")
        self.declare_parameter("shoot_done_topic", "/shoot_done")
        self.declare_parameter("follow_enable_topic", "/follow/enable")

        # salute_node의 stt sub/pub param
        self.declare_parameter("salute_accord_topic", "/salute_accord_topic")
        self.declare_parameter("salute_accord_out_topic", "/orchestrator/salute_accord_topic")
        self.declare_parameter("salute_heard_topic", "/salute_heard_text")
        self.declare_parameter("salute_heard_out_topic", "/orchestrator/salute_heard_text")

        # for node
        self.declare_parameter("auth_action_name", "/auth_action")
        self.declare_parameter("auth_attempts", 3)

        # trigger 기반 시작
        self.declare_parameter("trigger_topic", "/orchestrator/trigger")

        # lock_done 기반 자동 시작
        self.declare_parameter("lock_done_topic", "/follow/lock_done")
        self.declare_parameter("lock_done_debounce_sec", 2.0)

        # 기본 문어/답어(딕셔너리에서 못 찾으면 fallback)
        self.declare_parameter("challenge_text", "아이폰")
        self.declare_parameter("expected_text", "갤럭시")

        # DB/로그용 단계 이벤트 토픽 (로거가 타임스탬프를 찍음)
        self.declare_parameter("ui_event_topic", "/follow/ui_event")

        # 수정1)
        self.declare_parameter("saystop_text", "정지, 정지, 정지, 움직이면 쏜다!")

        # -------------------------
        # Read params
        # -------------------------
        self._start_topic = self.get_parameter("start_topic").value
        self._status_topic = self.get_parameter("status_topic").value
        self._auth_action_name = self.get_parameter("auth_action_name").value
        self._auth_attempts = int(self.get_parameter("auth_attempts").value)

        self._trigger_topic = self.get_parameter("trigger_topic").value

        self._lock_done_topic = self.get_parameter("lock_done_topic").value
        self._lock_done_debounce_sec = float(self.get_parameter("lock_done_debounce_sec").value)
        self._last_lock_done_start_t: float = 0.0

        self._salute_trigger_topic = self.get_parameter("salute_trigger_topic").value
        self._salute_done_topic = self.get_parameter("salute_done_topic").value
        self._shoot_trigger_topic = self.get_parameter("shoot_trigger_topic").value
        self._shoot_done_topic = self.get_parameter("shoot_done_topic").value
        self._follow_enable_topic = self.get_parameter("follow_enable_topic").value

        self._salute_accord_topic = self.get_parameter("salute_accord_topic").value
        self._salute_accord_out_topic = self.get_parameter("salute_accord_out_topic").value
        self._salute_heard_topic = self.get_parameter("salute_heard_topic").value
        self._salute_heard_out_topic = self.get_parameter("salute_heard_out_topic").value

        self._ui_event_topic = self.get_parameter("ui_event_topic").value

        # 수정2)
        self._saystop_text = str(self.get_parameter("saystop_text").value)

        # 기본 fallback
        self._challenge_default = str(self.get_parameter("challenge_text").value)
        self._expected_default = str(self.get_parameter("expected_text").value)

        # 런타임 적용값(시작 시 딕셔너리로 갱신)
        self._challenge = self._challenge_default
        self._expected = self._expected_default

        # state machine
        self._state = "IDLE"  # IDLE | AUTH | SALUTE_WAIT | SHOOT_WAIT
        self._busy = False
        self._attempts = 0

        # -------------------------
        # Pub/Sub
        # -------------------------
        self._pub_status = self.create_publisher(String, self._status_topic, 10)
        self._pub_ui_event = self.create_publisher(String, self._ui_event_topic, 10)

        self._sub_start = self.create_subscription(
            String, self._start_topic, self._on_start, 10, callback_group=self._cbg
        )
        self._sub_trigger = self.create_subscription(
            Bool, self._trigger_topic, self._on_trigger, 10, callback_group=self._cbg
        )

        # lock_done latched-like QoS (publisher와 동일하게)
        self._qos_lock_done = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub_lock_done = self.create_subscription(
            Bool,
            self._lock_done_topic,
            self._on_lock_done,
            self._qos_lock_done,
            callback_group=self._cbg,
        )

        self._pub_follow_enable = self.create_publisher(Bool, self._follow_enable_topic, 10)
        self._pub_salute_trigger = self.create_publisher(Bool, self._salute_trigger_topic, 10)
        self._pub_shoot_trigger = self.create_publisher(Bool, self._shoot_trigger_topic, 10)

        # salute_node stt republish
        self._pub_salute_accord_text = self.create_publisher(String, self._salute_accord_out_topic, 10)
        self._pub_salute_heard_text = self.create_publisher(String, self._salute_heard_out_topic, 10)

        # salute/shoot done subscribers
        self._sub_salute_done = self.create_subscription(
            Bool, self._salute_done_topic, self._on_salute_done, 10, callback_group=self._cbg
        )
        self._sub_shoot_done = self.create_subscription(
            Bool, self._shoot_done_topic, self._on_shoot_done, 10, callback_group=self._cbg
        )

        # salute_node STT subs
        self._last_salute_accord_text = ""
        self._last_salute_heard_text = ""
        self._sub_salute_accord_text = self.create_subscription(
            String, self._salute_accord_topic, self._on_salute_accord_text, 10, callback_group=self._cbg
        )
        self._sub_salute_heard_text = self.create_subscription(
            String, self._salute_heard_topic, self._on_salute_heard_text, 10, callback_group=self._cbg
        )

        self._auth = ActionClient(self, Auth, self._auth_action_name, callback_group=self._cbg)

        # 초기 문어/답어 갱신 + 상태
        self._refresh_phrase_for_today(force_log=True)
        self._set_status("Ready")
        self._emit_event("시스템 준비 완료")

    # -------------------------
    # Event helper (DB/로그용) - 시간은 로거에서 찍음
    # -------------------------
    def _emit_event(self, text: str) -> None:
        try:
            self._pub_ui_event.publish(String(data=str(text)))
        except Exception:
            return

    # -------------------------
    # Phrase selection (dict)
    # -------------------------
    @staticmethod
    def _today_key() -> str:
        now = datetime.datetime.now(ZoneInfo("Asia/Seoul"))
        return now.strftime("%Y-%m-%d")

    def _refresh_phrase_for_today(self, *, force_log: bool = False) -> None:
        key = self._today_key()

        if key in WEEKLY_PHRASES:
            ch, ex = WEEKLY_PHRASES[key]
            ch = str(ch).strip()
            ex = str(ex).strip()
            if ch and ex:
                changed = (ch != self._challenge) or (ex != self._expected)
                self._challenge, self._expected = ch, ex
                if force_log or changed:
                    self.get_logger().info(
                        f"[PHRASE] date={key} challenge='{self._challenge}' expected='{self._expected}'"
                    )
                    self._emit_event(f"오늘의 문어: {self._challenge} / 답어: {self._expected}")
                return

        # fallback
        changed = (self._challenge != self._challenge_default) or (self._expected != self._expected_default)
        self._challenge, self._expected = self._challenge_default, self._expected_default
        if force_log or changed:
            self.get_logger().info(
                f"[PHRASE] date={key} not found -> fallback challenge='{self._challenge}' expected='{self._expected}'"
            )
            self._emit_event(f"오늘의 문어: {self._challenge} / 답어: {self._expected}")
    
    # 수정3)
    def _say(self, text: str):
        if self._saystop_text and text and text.strip():
            speak(text)

    # -------------------------
    # Start triggers
    # -------------------------
    def _on_start(self, msg: String):
        if self._busy:
            self._set_status("Busy, ignoring start")
            self._emit_event("이미 진행 중 → 시작 무시")
            return

        # 수동 start 메시지로 expected만 디버그 override 유지
        txt = (msg.data or "").strip()
        if txt:
            self._expected = txt
            self._emit_event("수동 시작 → 답어 변경 후 인증 시작")
        else:
            self._emit_event("수동 시작 → 인증 시작")

        self._start_sequence()

    def _on_trigger(self, msg: Bool):
        if not msg.data:
            return
        if self._busy:
            self._set_status("Busy, ignoring trigger")
            self._emit_event("이미 진행 중 → 시작 무시")
            return

        self._emit_event("외부 트리거 → 인증 시작")
        self._start_sequence()

    def _on_lock_done(self, msg: Bool):
        if not msg.data:
            return

        # IDLE에서만 lock_done을 시작 트리거로 인정
        if self._state != "IDLE":
            return

        if self._busy:
            return

        now = time.time()
        if (now - self._last_lock_done_start_t) < self._lock_done_debounce_sec:
            return
        self._last_lock_done_start_t = now

        self._set_status("LOCK_DONE -> start AUTH")
        self._emit_event("대상 락온 완료 → 인증 시작")
        
        # 수정4)
        self._say(self._saystop_text)
        self.get_logger().info("정지 정지 정지")

        self._start_sequence()

    # -------------------------
    # Core sequence
    # -------------------------
    def _start_sequence(self):
        # 매 시작 시점에 오늘 문어/답어 갱신(자정 넘어가도 반영)
        self._refresh_phrase_for_today(force_log=False)

        self._busy = True
        self._state = "AUTH"
        self._attempts = 0

        if not str(self._expected).strip():
            self._set_status("Expected text is empty")
            self._emit_event("답어가 비어있음 → 인증 중단")
            self._busy = False
            self._state = "IDLE"
            return

        self._set_status("Start !!!")
        self.get_logger().info(f"challenge='{self._challenge}', expected='{self._expected}'")
        self._emit_event(f"인증 시작 (문어: {self._challenge} / 답어: {self._expected})")

        # AUTH 동안은 follow ON 유지(요구사항)
        self._set_follow_enable(True)
        self._emit_event("추적 유지 (인증 중)")

        self._try_auth()

    def _try_auth(self):
        self._attempts += 1
        self._set_status(f"Authentication : {self._attempts} / {self._auth_attempts}")
        self._emit_event(f"수어 {self._attempts} 시작")

        ok = False
        for i in range(5):
            if self._auth.wait_for_server(timeout_sec=1.0):
                ok = True
                break
            self._set_status(f"Waiting auth server... {i+1}/{5}")

        if not ok:
            self._set_status("Auth server not available")
            self._emit_event("인증 서버 응답 없음 → 인증 중단")
            self._set_follow_enable(True)
            self._busy = False
            self._state = "IDLE"
            return

        goal = Auth.Goal()
        goal.challenge = self._challenge
        goal.expected = self._expected

        future = self._auth.send_goal_async(goal, feedback_callback=self._auth_feedback)
        future.add_done_callback(self._auth_goal_response)

    def _auth_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._set_status("Auth goal rejected")
            self._emit_event("인증 요청이 거절됨 → 인증 중단")
            self._set_follow_enable(True)
            self._busy = False
            self._state = "IDLE"
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._auth_result)

    def _auth_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f"auth result: success={result.success} "
            f"heard='{result.heard_text}' "
            f"code={result.code} "
            f"reason='{result.reason}'"
        )

        ok_txt = "성공" if result.success else "실패"
        heard = (result.heard_text or "").strip()
        if heard:
            self._emit_event(f"수어 {self._attempts} 결과: {ok_txt} (인식: '{heard}')")
        else:
            self._emit_event(f"수어 {self._attempts} 결과: {ok_txt}")

        if result.success:
            self._set_status("Auth success -> SALUTE")
            self._state = "SALUTE_WAIT"

            # salute 직전만 follow OFF
            self._set_follow_enable(False)
            self._emit_event("추적 중지")

            self._pub_salute_trigger.publish(Bool(data=True))
            self._emit_event("경례 동작 실행")
            return

        if self._attempts < self._auth_attempts:
            self._emit_event("다음 수어 시도 진행")
            self._try_auth()
        else:
            self._set_status("Auth failed 3 times -> SHOOT")
            self._state = "SHOOT_WAIT"

            # shoot 직전만 follow OFF
            self._set_follow_enable(False)
            self._emit_event("추적 중지")

            self._pub_shoot_trigger.publish(Bool(data=True))
            self._emit_event("사격 동작 실행")
            return

    def _auth_feedback(self, fb_msg):
        fb = fb_msg.feedback
        self._set_status(f"Auth mode: {fb.mode}")
        # 사람친화 문구는 결과 중심이라, 피드백은 UI 이벤트로는 생략(원하면 주석 해제)
        # self._emit_event(f"인증 진행 중: {fb.mode}")

    # -------------------------
    # Done callbacks
    # -------------------------
    def _on_salute_done(self, msg: Bool):
        if self._state != "SALUTE_WAIT":
            return
        self._set_status(f"Salute done: {'OK' if msg.data else 'FAIL'}")

        done_txt = "성공" if msg.data else "실패"
        self._emit_event(f"경례 동작 완료 ({done_txt})")

        self._set_follow_enable(True)
        self._emit_event("추적 재개")

        self._state = "IDLE"
        self._busy = False

    def _on_shoot_done(self, msg: Bool):
        if self._state != "SHOOT_WAIT":
            return
        self._set_status(f"Shoot done: {'OK' if msg.data else 'FAIL'}")

        done_txt = "성공" if msg.data else "실패"
        self._emit_event(f"사격 동작 완료 ({done_txt})")

        self._set_follow_enable(True)
        self._emit_event("추적 재개")

        self._state = "IDLE"
        self._busy = False

    # -------------------------
    # salute_node STT republish
    # -------------------------
    def _on_salute_accord_text(self, msg: String):
        text = (msg.data or "").strip()
        self._last_salute_accord_text = text
        self._pub_salute_accord_text.publish(String(data=text))

    def _on_salute_heard_text(self, msg: String):
        text = (msg.data or "").strip()
        self._last_salute_heard_text = text
        self._pub_salute_heard_text.publish(String(data=text))

    # -------------------------
    # Status + follow enable
    # -------------------------
    def _set_status(self, txt: str):
        self._pub_status.publish(String(data=str(txt)))

    def _set_follow_enable(self, enabled: bool) -> None:
        self._pub_follow_enable.publish(Bool(data=bool(enabled)))


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
