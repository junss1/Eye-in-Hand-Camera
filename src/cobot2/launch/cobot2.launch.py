from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Doosan (real) bringup ---
    
    REAL_SWITCH = True

    REAL = {"mode": "real",    "host": "192.168.1.100", "port": "12345", "model": "m0609"}
    VIRTUAL = {"mode": "virtual", "host": "127.0.0.1",     "port": "12345", "model": "m0609"}
    launch_args = (REAL if REAL_SWITCH else VIRTUAL)
    dsr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dsr_bringup2"),
                "launch",
                "dsr_bringup2_rviz.launch.py",
            )
        ),
        launch_arguments=launch_args.items(),
    )

    # --- Perception (optional, no hard dependency for motion) ---
    yolo_camera_node = Node(
        package="cobot2",
        executable="yolo_camera",
        name="yolo_camera_node",
        output="screen",
        parameters=[{
            "image_topic": "/camera/camera/color/image_raw",
            "show_debug": False,

            # UI outputs
            "publish_annotated": True,
            "annotated_topic": "/follow/annotated_image",
            "ui_event_topic": "/follow/ui_event",

            # lock_done topic
            "lock_done_topic": "/follow/lock_done",
        }],
    )

    # --- DB Logger (SQLite + snapshots) ---
    follow_logger_node = Node(
        package="cobot2",
        executable="follow_logger_node",
        name="follow_logger_node",
        output="screen",
        parameters=[{
            "ui_event_topic": "/follow/ui_event",
            "annotated_image_topic": "/follow/annotated_image",
            "log_root_dir": os.path.join(os.path.expanduser("~"), "logs", "follow_runs"),
            "snapshot_enable": True,
        }],
    )

    # --- UI (Live + History) ---
    follow_ui_node = Node(
        package="cobot2",
        executable="follow_ui_node",
        name="follow_ui_node",
        output="screen",
        parameters=[{
            "annotated_image_topic": "/follow/annotated_image",
            "ui_event_topic": "/follow/ui_event",
        }],
    )


    # --- Authentication Action Server (must be ready before orchestrator sends goal) ---
    auth_action_server = Node(
        package="cobot2",
        executable="auth_action",
        name="auth_action_server",
        output="screen",
    )

    # --- Motion nodes (must be ready before orchestrator publishes triggers) ---
    salute_node = Node(
        package="cobot2",
        executable="salute",
        name="salute_node",
        output="screen",
    )
    shoot_node = Node(
        package="cobot2",
        executable="shoot",
        name="shoot_node",
        output="screen",
    )

    # --- Follow control ---
    tcp_follow_node = Node(
        package="cobot2",
        executable="tcp_follow",
        name="tcp_follow_node",
        output="screen",
        parameters=[{
            "error_topic": "/follow/error_norm",
            "enable_topic": "/follow/enable",
            "follow_enable_default": True,
        }],
    )

    # --- Orchestrator (start last) ---
    orchestrator_node = Node(
        package="cobot2",
        executable="orchestrator",
        name="orchestrator",
        output="screen",
    )
    # --- safety moitor (start last) ---
    safety_monitor_node = Node(
        package="cobot2",
        executable="safety_monitor",
        name="safety_monitor_node",
        output="screen",
        parameters=[{
            "robot_id": "dsr01",
            "robot_model": "m0609",
            "event_topic": "/safety/event",
            "poll_period_sec": 0.3,
            "stop_on_fault": True,
        }],
    )

    # ========= Launch ordering (time-based gating) =========
    # 0s  : dsr bringup
    # 1.5s: auth + motion nodes (they may wait for dsr services internally)
    # 3.0s: follow node (disabled)
    # 4.0s: orchestrator (now safe to trigger)
    return LaunchDescription([
        dsr_launch,

        # perception can start anytime; keep it early
        TimerAction(period=10., actions=[yolo_camera_node]),

        # yolo 이후에 logger/ui 붙이기
        TimerAction(period=12., actions=[follow_logger_node, follow_ui_node]),

        # servers/consumers before orchestrator
        TimerAction(period=13., actions=[auth_action_server, salute_node, shoot_node]),

        # safety_monitor last
        TimerAction(period=14., actions=[safety_monitor_node]),

        # follow after motion nodes, still disabled
        TimerAction(period=16., actions=[tcp_follow_node]),

        # orchestrator last
        TimerAction(period=20., actions=[orchestrator_node]),
    ])