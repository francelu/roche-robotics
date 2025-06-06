#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.action import Undock, RotateAngle, DriveDistance

DRIVE_DISTANCE = 0.25  # Default distance for drive actions in meters
ROTATE_ANGLE = 0.523598  # Default angle for rotate actions in radians (30 degrees)

class RobotActionExecutor(Node):
    def __init__(self):
        super().__init__('robot_action_executor')
        self.publisher_action_completed = self.create_publisher(String, 'action_completed', 10)
        self.subscription_action = self.create_subscription(String, 'robot_action', self.callback_robot_action, 10)

        self.action_in_progress = False

    def callback_robot_action(self, message):
        if self.action_in_progress:
            self.get_logger().info("Action in progress. Other actions are ignored.")
            return

        action = message.data.strip().lower()
        self.get_logger().info(f"Action received: {action}")
        self.action_in_progress = True

        match action:
            case "undock":
                self.undock_action()
            case "move_forward":
                self.drive_distance_action(DRIVE_DISTANCE)
            case "move_backward":
                self.drive_distance_action(-DRIVE_DISTANCE)
            case "rotate_left":
                self.call_rotate_action(ROTATE_ANGLE)
            case "rotate_right":
                self.call_rotate_action(-ROTATE_ANGLE)
            case _:
                self.get_logger().warn(f"Unknown action: {action}")
                self.action_in_progress = False

    # =========================================================
    # Undock action, response and result handling
    def undock_action(self):
        self.undock_action_client = ActionClient(self, Undock, 'undock')

        self.get_logger().info("Waiting for Undock action server...")
        if not self.undock_action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Undock server unavailable!")
            self.action_in_progress = False
            return

        self.get_logger().info("Sending undock command...")
        undock_message = Undock.Goal()
        future = self.undock_action_client.send_goal_async(undock_message)
        future.add_done_callback(self.undock_response)

    def undock_response(self, future):
        undock_response = future.result()
        if not undock_response.accepted:
            self.get_logger().info("Undock action rejected.")
            self.action_in_progress = False
            return
        
        self.get_logger().info("Undock action accepted, waiting for result...")
        undock_response.get_result_async().add_done_callback(self.undock_result)

    def undock_result(self, future):
        self.get_logger().info("Undock action completed.")
        self.action_in_progress = False
        result_message = String()
        result_message.data = "undock_completed"
        self.publisher_action_completed.publish(result_message)
    # =========================================================
    
    # =========================================================
    # Drive distance action, response and result handling
    def drive_distance_action(self, distance):
        self.drive_distance_action_client = ActionClient(self, DriveDistance, 'drive_distance')
        self.get_logger().info("Waiting for DriveDistance action server...")
        if not self.drive_distance_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("DriveDistance server unavailable!")
            self.action_in_progress = False
            return

        self.get_logger().info(f"Sending DriveDistance command ({distance} m)...")
        drive_distance_message = DriveDistance.Goal()
        drive_distance_message.distance = distance
        future = self.drive_distance_action_client.send_goal_async(drive_distance_message)
        future.add_done_callback(self.drive_goal_response)

    def drive_goal_response(self, future):
        drive_distance_response = future.result()
        if not drive_distance_response.accepted:
            self.get_logger().info("DriveDistance action rejected.")
            self.action_in_progress = False
            return
        
        self.get_logger().info("DriveDistance action accepted, waiting for result...")
        drive_distance_response.get_result_async().add_done_callback(self.drive_result_callback)

    def drive_result_callback(self, future):
        self.get_logger().info("DriveDistance action completed.")
        self.action_in_progress = False
        result_message = String()
        result_message.data = "drive_completed"
        self.publisher_action_completed.publish(result_message)
    # =========================================================

    # =========================================================
    # Rotate angle action, response and result handling
    def call_rotate_action(self, angle):
        self.rotate_action_client = ActionClient(self, RotateAngle, 'rotate_angle')
        self.get_logger().info("Waiting for RotateAngle action server...")
        if not self.rotate_action_client.wait_for_server(timeout_sec=4.0):
            self.get_logger().error("RotateAngle server unavailable!")
            self.action_in_progress = False
            return

        self.get_logger().info(f"Sending RotateAngle command ({angle} rad)...")
        rotate_message = RotateAngle.Goal()
        rotate_message.angle = angle
        future = self.rotate_action_client.send_goal_async(rotate_message)
        future.add_done_callback(self.rotate_goal_response_callback)

    def rotate_goal_response_callback(self, future):
        rotate_response = future.result()
        if not rotate_response.accepted:
            self.get_logger().info("RotateAngle action rejected.")
            self.action_in_progress = False
            return
        
        self.get_logger().info("RotateAngle action accepted, waiting for result...")
        rotate_response.get_result_async().add_done_callback(self.rotate_result_callback)

    def rotate_result_callback(self, future):
        self.get_logger().info("RotateAngle action completed.")
        self.action_in_progress = False
        result_message = String()
        result_message.data = "rotate_completed"
        self.publisher_action_completed.publish(result_message)
    # =========================================================

def main(args=None):
    rclpy.init(args=args)
    node = RobotActionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rlcpy.shutdown()

if __name__ == '__main__':
    main()

