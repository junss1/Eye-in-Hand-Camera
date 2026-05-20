import rclpy
import DR_init
import time
from dsr_msgs2.srv import SetRobotControl # ★ 핵심: 제어 상태 강제 변환 서비스
from std_msgs.msg import Bool

# 로봇 설정 (환경에 맞게 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 40
ACC = 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# [추가] 제어 명령 상수
CONTROL_RESET_SAFE_STOP = 2  # 보호 정지 해제
CONTROL_RESET_SAFE_OFF = 3   # 서보 켜기 (Safe Off -> Standby)

# ---------------------------------------------------------
# 로봇 상태 코드 매핑 테이블 (매뉴얼 기준)
# ---------------------------------------------------------
ROBOT_STATE_MAP = {
    0: "STATE_INITIALIZING (초기화 중)",
    1: "STATE_STANDBY (대기 중 - 정상)",
    2: "STATE_MOVING (이동 중)",
    3: "STATE_SAFE_OFF (서보 꺼짐)",
    4: "STATE_TEACHING (티칭 모드)",
    5: "STATE_SAFE_STOP (안전 정지 - 외부 충격 등)",
    6: "STATE_EMERGENCY_STOP (비상 정지)",
    7: "STATE_HOMMING (호밍 중)",
    8: "STATE_RECOVERY (복구 모드)",
    9: "STATE_SAFE_STOP2 (안전 정지 2)",
    10: "STATE_SAFE_OFF2 (서보 꺼짐 2)",
    11: "STATE_RESERVED1",
    12: "STATE_RESERVED2",
    13: "STATE_RESERVED3",
    14: "STATE_RESERVED4",
    15: "STATE_NOT_READY (준비 안 됨)"
}

def call_set_robot_control(control_value):
    """로봇 제어 상태를 강제로 리셋하거나 변경하는 함수"""
    node = DR_init.__dsr__node
    srv_name = f'/{ROBOT_ID}/system/set_robot_control'
    cli = node.create_client(SetRobotControl, srv_name)
    
    if not cli.wait_for_service(timeout_sec=1.0):
        print(f"[Err] {srv_name} 서비스를 찾을 수 없습니다.")
        return False

    req = SetRobotControl.Request()
    req.robot_control = control_value
    
    future = cli.call_async(req)
    
    # 결과 대기 (블로킹 없이 처리하기 위해 spin_once 사용)
    start_wait = time.time()
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.01)
        
        # [수정] 타임아웃을 3.0초 -> 5.0초로 증가 (컨트롤러 처리 지연 대비)
        if time.time() - start_wait > 5.0: 
            print("[Err] 서비스 호출 시간 초과 (컨트롤러 응답 지연)")
            return False

    try:
        res = future.result()
        return res.success
    except Exception as e:
        print(f"[Err] 서비스 호출 실패: {e}")
        return False

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import get_robot_mode, set_robot_mode, set_safe_stop_reset_type

    print(">>> Initializing robot settings...")

    # # [추가] 안전 정지 리셋 타입 설정 (0: Program Stop)
    # set_safe_stop_reset_type(0)

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행 (안전성 확보)
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)  # 설정 안정화를 위해 잠시 대기
    
    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}") 
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

def perform_task(pub_safety):
    """로봇이 수행할 작업 (상태 모니터링 및 복구)"""
    print("Performing task...")
    # 상태 조회 및 제어 함수 임포트
    from DSR_ROBOT2 import get_robot_state, drl_script_stop, DR_QSTOP_STO,get_last_alarm

    print(f"[{ROBOT_ID}] 로봇 상태 모니터링 시작")
    print("=" * 60)

    while True:
        # 1. 상태값(숫자) 조회
        state_code = get_robot_state()
        
        # 2. 딕셔너리에서 상태 의미(문자열) 찾기
        state_desc = ROBOT_STATE_MAP.get(state_code, "UNKNOWN_STATE")
        
        # 3. 상태에 따른 분기 처리
        if state_code == 1:
            # 정상 상태
            print(f">>> [Normal] Current Robot State: [{state_code}] -> {state_desc}", end='\r')
        
        elif state_code == 5:
            # [복구 로직] STATE_SAFE_STOP (안전 정지)
            print(f"\n!!! [Warning] 보호 정지 감지 ({state_desc}) !!!")
            print("!!! 복구를 시도합니다. (외력이 제거되어야 합니다) !!!")
            pub_safety.publish(Bool(data=True))
            
            # 1. 스크립트 정지 (안전 확보)
            drl_script_stop(DR_QSTOP_STO)

            time.sleep(1.)  # 외력 제거 대기
            
            # 2. ★ SetRobotControl(2)로 강제 리셋 시도
            if call_set_robot_control(CONTROL_RESET_SAFE_STOP): # 리셋 명령 전송
                print("   -> 리셋 명령 전송됨. 복구 대기 중...")
                # time.sleep(2.0)
                if get_robot_state() == 1:
                    print("   -> [Success] 보호 정지 해제. 설정을 다시 로드합니다.")
                    initialize_robot() # 초기화 재수행
            else:
                print("   -> 리셋 실패.")
                time.sleep(1.0)

            # -------------------------------------------------------------
            # [CASE 2] STATE_SAFE_OFF (3) - 서보 꺼짐 (복구 시퀀스 적용)
            # -------------------------------------------------------------
        elif state_code == 3:
            # [복구 로직] STATE_SAFE_OFF (서보 꺼짐)
            print(f"\n*** [Error] 서보 꺼짐 감지 ({state_desc}) ***")
            print("*** 서보 ON (Reset Safe Off)을 시도합니다. ***")
            
            # 1. 기존 스크립트 정지 (필수)
            drl_script_stop(DR_QSTOP_STO)
            time.sleep(0.5)
            
            # 2. ★ SetRobotControl(3)으로 서보 ON 시도
            if call_set_robot_control(CONTROL_RESET_SAFE_OFF):
                print("   -> 서보 ON 명령 전송됨. 기동 대기 중...")
                #                                  time.sleep(2.0)
                if get_robot_state() == 1:
                    print("   -> [Success] 서보 ON 완료.")
                    initialize_robot() # 초기화 재수행
            else:
                # 여전히 3번 상태라면 하드웨어 스위치 문제
                alarm = get_last_alarm()
                print(f">>> [Fail] 서보 ON 실패. 현재 상태: {state_desc}")
                if alarm:
                    print(f"   - 거절 사유(알람): {alarm}")
                print("   !!! 조치 필요: [비상정지 버튼] 해제 또는 [티칭 펜던트 스위치(Auto)] 확인 필요 !!!\n")
                time.sleep(2) 
                
        else:
            # 그 외 상태
            print(f">>> [Check] Current Robot State: [{state_code}] -> {state_desc}")

        # 0.5초 간격으로 반복
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("check_robot_state", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    pub_safety = node.create_publisher(Bool, "/orchestrator/safety_stop", 10)


    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행 (한 번만 호출)
        perform_task(pub_safety)

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()