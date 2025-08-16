#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from geometry_msgs.msg import PoseStamped


class PathValidationNode(Node): 
    def __init__(self):
        super().__init__("path_validation") 
        self.is_path_available_ = None
        self.compute_path_to_pose_client_ = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")


    def send_goal(self, start_pose: PoseStamped, goal_pose: PoseStamped):
            self.compute_path_to_pose_client_.wait_for_server()

            goal = ComputePathToPose.Goal()
            goal.start = start_pose
            goal.goal = goal_pose

            # self.get_logger(). \
            #     info(f"Send goal with position ({position_x}, {position_y}) and orientation {orientation_z}")
            self.compute_path_to_pose_client_. \
            send_goal_async(goal). \
            add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
            self.goal_handle_: ClientGoalHandle = future.result()
            if self.goal_handle_.accepted:
                self.get_logger().info("Goal got accepted")
                self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
            else:
                self.get_logger().warn("Goal got rejected")
                self.is_path_available_ = False


    def goal_result_callback(self, future):
        status = future.result().status
        result: ComputePathToPose.Result = future.result().result
        path = result.path
        if not path.poses:
            self.get_logger().info("Planner couldn't find a path to the goal.")
            self.is_path_available_ = False
        else: 
            self.is_path_available_ = True
        # if status == GoalStatus.STATUS_SUCCEEDED:
        #     self.get_logger().info("Succeeded")
        # elif status == GoalStatus.STATUS_ABORTED:
        #     self.get_logger().error("Aborted")
        # elif status == GoalStatus.STATUS_CANCELED:
        #     self.get_logger().warn("Canceled")


def main(args=None):
    rclpy.init(args=args)
    node = PathValidationNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
