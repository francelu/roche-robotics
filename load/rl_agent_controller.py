#!/usr/bin/env python3
import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from stable_baselines3 import PPO

# Note: adjust the model path as needed
MODEL_PATH = "/home/flu/roche/ppo_random_ball_and_robot_pos_rgb_v01.zip"

class RLAgentController(Node):
    def __init__(self):
        super().__init__("rl_agent_controller")

        self.get_logger().info(f"Loading PPO model from: {MODEL_PATH}")
        self.model = PPO.load(MODEL_PATH, device='cuda', custom_objects={"n_steps": 1, "n_envs": 1})
        self.get_logger().info("Model loaded.")

        # ROS setup
        self.publisher_action = self.create_publisher(String, "robot_action", 10)
        self.subscription_action_completed = self.create_subscription(String, "action_completed", self.callback_action_completed, 10)
        self.subscription_image = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.callback_image, 10)

        self.cv_bridge = CvBridge()
        self.action_in_progress = False
        self.image = None

        # Note : either undock by hand or use the following line to undock automatically
        self.send_action("undock")

    def callback_image(self, msg: Image):
        raw_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image = cv2.resize(raw_image, (256, 256), interpolation=cv2.INTER_LINEAR)
        image = image.astype(np.float32) / 255.0
        self.image = image
    
        self.predict()
    
    def callback_action_completed(self, _msg: String):
        self.action_in_progress = False
        time.sleep(0.5)  # Try to fix the orange light issue

        self.predict()
    
    def predict(self):
        if self.action_in_progress or self.image is None:
            return

        action_id, _ = self.model.predict(self.image, deterministic=True)
        action_id = int(action_id)
        action = self.map_id_to_action(action_id)
        self.send_action(action)

    def send_action(self, action: str):
        self.get_logger().info(f"Sending command: {action}")
        message = String()
        message.data = action
        self.publisher_action.publish(message)
        self.action_in_progress = True

    @staticmethod
    def map_id_to_action(a: int) -> str:
        return {
            0: "move_forward",
            1: "move_backward",
            2: "rotate_left",
            3: "rotate_right",
            4: "done"
        }.get(a, "move_forward")

def main(args=None):
    rclpy.init(args=args)
    node = RLAgentController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Interrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rlcpy.shutdown()

if __name__ == "__main__":
    main()