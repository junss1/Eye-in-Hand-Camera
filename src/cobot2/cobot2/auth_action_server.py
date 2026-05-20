import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from ament_index_python.packages import get_package_share_directory

import os
from dotenv import load_dotenv

# class STT, get_keyword
from cobot2.stt import STT
from cobot2.get_keyword import GetKeyword
from cobot2.speak import speak

# user define interfaces - Auth is Authentication
from cobot2_interfaces.action import Auth


class AuthServer(Node):

    def __init__(self):
        ############ Node initialize ############
        #########################################
        super().__init__("auth_server")

        pkg = get_package_share_directory("cobot2")
        env_path = os.path.join(pkg, "resource", ".env")

        if not os.path.exists(env_path):
            self.get_logger().error(f".env not found: {env_path}")
        else:
            load_dotenv(env_path)

        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            # self.get_logger().error("OPENAI_API_KEY not set (dotenv load failed or missing key)")
            raise RuntimeError("OPENAI_API_KEY not set (dotenv load failed or missing key)")


        self.stt = STT(api_key)
        self.verifier = GetKeyword(temperature=0.3)

        self.server = ActionServer(
            self,
            Auth,
            "auth_action",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            # must add qos setting !!!
        )

        self.get_logger().info("Auth Action Server Ready")

    ############ helper function / start
    ########################################################################
    # repllace _norm func made by "Jo" ?????
    # used, import re
    def _norm(self, s):
        return "".join(s.split()).lower()
    
    def _check_cancel(self, goal_handle, res):
        if goal_handle.is_cancel_requested:
            res.success = False
            res.code = 3  # CANCELED
            return True
        return False

    ############ helper function / END
    ########################################################################




    ############ call back functions / start
    ########################################################################
    def goal_cb(self, goal):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):

        g = goal_handle.request
        fb = Auth.Feedback()
        res = Auth.Result()

        ############ ask challenge word ############
        ############################################
        self.get_logger().info(f"ASK: {g.challenge}")
        
        # if cancel
        if self._check_cancel(goal_handle, res):
            goal_handle.canceled()
            return res

        # 0: OK / 1: TIMEOUT / 2: MISMATCH / 3: CANCELED
        
        # READY
        fb.mode = 0
        goal_handle.publish_feedback(fb)
        speak(g.challenge)

        # WAITING
        fb.mode = 1
        goal_handle.publish_feedback(fb)

        # LISTENING
        fb.mode = 2
        goal_handle.publish_feedback(fb)
        try:
            heard = self.stt.speech2text()
        except Exception as e:
            self.get_logger().error(str(e))
            heard = ""
        
        # if cancel
        if self._check_cancel(goal_handle, res):
            goal_handle.canceled()
            return res
        
        # VERIFYING
        fb.mode = 3
        goal_handle.publish_feedback(fb)

        # if v is empty, verifier return TIMEOUT
        v = self.verifier.verify(expected=g.expected, heard_text=heard)

        res.success = bool(v["success"])
        res.heard_text = heard
        res.code = int(v["code"])
        res.reason = str(v["reason"])

        goal_handle.succeed()
        return res
    ############ call back functions / END
    ########################################################################


def main():
    rclpy.init()
    node = AuthServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
