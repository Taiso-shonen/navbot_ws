#!/usr/bin/env python3
import time
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from example_interfaces.msg import String
from std_srvs.srv import Trigger
from my_navbot_interfaces.srv import SetPose
import tf_transformations


class NavToPoseClientNode(Node): 
    def __init__(self):
        super().__init__("nav_to_pose_client")
        self.declare_parameter("set_initial_pose", False)
        self.set_initial_pose_ = self.get_parameter("set_initial_pose").value
        self.goal_handle_ = None
        self.nav_to_pose_client_ = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.initial_pose_pub_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_availability_pub_ = self.create_publisher(String, 'goal_availability', 10) 
        self.goal_response_pub_ = self.create_publisher(String, 'goal_response', 10) 
        self.goal_result_pub_ = self.create_publisher(String, 'goal_result', 10) 
        self.sending_goal_server_ = self.create_service(SetPose, 'send_goal', self.callback_sending_goal)
        self.canceling_goal_server_ = self.create_service(Trigger, 'cancel_goal', self.callback_canceling_goal)
        self.timer_ = self.create_timer(1.0, self.publish_goal_availability, autostart=False)


    def send_initial_pose(self, position_x, position_y, orientation_z):
        if self.set_initial_pose_:
            pose = self.create_initial_pose_stamped(position_x, position_y, orientation_z)
            while self.initial_pose_pub_.get_subscription_count() == 0:
                self.get_logger().info("Waiting for a subscriber to /initialpose...")
                time.sleep(1)

            retry_count = 0
            max_retries = 5

            while retry_count < max_retries:
                self.initial_pose_pub_.publish(pose)
                time.sleep(0.5)
                self.initial_pose_pub_.publish(pose)
                if self.initial_pose_pub_.wait_for_all_acked(Duration(seconds=1)):
                    self.get_logger().info("Initial pose acknowledged.")
                    break 
                else:
                    self.get_logger().warn("Initial pose not yet acknowledged, retrying...")
                    retry_count += 1
                    time.sleep(0.5)

            if retry_count >= max_retries:
                self.get_logger().error("Failed to confirm initial pose.")
        else:
            self.timer_.reset()



    def publish_goal_availability(self):
        self.nav_to_pose_client_.wait_for_server()
        msg = String()
        msg.data = "Send a new goal"
        self.goal_availability_pub_.publish(msg)

    # Added this fuction
    def publish_goal_response(self, message):
        msg = String()
        msg.data = message
        self.goal_response_pub_.publish(msg)

    # Added this fuction
    def publish_goal_result(self, message):
        msg = String()
        msg.data = message
        self.goal_result_pub_.publish(msg)
                
    def callback_sending_goal(self, request: SetPose.Request, response: SetPose.Response):
        self.get_logger().info("Received a request")
        if not self.timer_.is_canceled():
            self.timer_.cancel()
            
        x = request.x
        y = request.y
        theta = request.theta
        print(x, y, theta)
        self.send_goal(x, y, theta)

        response.success = True
        response.msg = "Accepted."

        return response
    
    def send_goal(self, position_x, position_y, orientation_z):
        while not self.nav_to_pose_client_.wait_for_server(1.0):
            self.get_logger().warn("Waiting for server...")

        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose_stamped(position_x, position_y, orientation_z)
        goal.behavior_tree = ""

        self.get_logger(). \
            info(f"Send goal with position ({position_x}, {position_y}) and orientation {orientation_z}")
        self.nav_to_pose_client_. \
        send_goal_async(goal). \
        add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            message = "Goal got accepted"
            self.get_logger().info(message)
            self.publish_goal_response(message)
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            message = "Goal got rejected"
            self.get_logger().warn(message)
            self.publish_goal_response(message)
        

    def goal_result_callback(self, future):
        status = future.result().status
        result: NavigateToPose.Result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            message = "Succeeded"
            self.get_logger().info(message)
        elif status == GoalStatus.STATUS_ABORTED:
            message = "Aborted"
            self.get_logger().error(message)
        elif status == GoalStatus.STATUS_CANCELED:
            message = "Canceled"
            self.get_logger().warn(message)
        if not result.error_code == 0:
            self.get_logger().info("Error code: " + str(result.error_code))
            self.get_logger().info("Error msg: " + str(result.error_msg))
        
        self.publish_goal_result(message)


    def goal_feedback_callback(self, feedback_msg):
        feedback: NavigateToPose.Feedback = feedback_msg.feedback
        position_x = feedback.current_pose.pose.position.x
        position_y = feedback.current_pose.pose.position.y
        time_remaining = feedback.estimated_time_remaining.sec
        distance_remaining =feedback.distance_remaining

        self.get_logger().info(f"Current position: ({position_x}, {position_y})")
        self.get_logger().info(f"Remaining distance: {distance_remaining:.2f} meters")
        self.get_logger().info(f"Estimated time: {time_remaining:.2f} seconds")


    def callback_canceling_goal(self, request, response: Trigger.Response):
        self.get_logger().info("Received a cancel request")
        self.cancel_goal()
        response.success = True
        response.message = "Cancel request accepted."
            
        return response

    def cancel_goal(self):
        if self.goal_handle_ is not None:
            self.get_logger().info("Send a cancel request")
            self.goal_handle_.cancel_goal_async()


    def create_initial_pose_stamped(self, position_x, position_y, orientation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = position_x
        pose.pose.pose.position.y = position_y
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.x = q_x
        pose.pose.pose.orientation.y = q_y
        pose.pose.pose.orientation.z = q_z
        pose.pose.pose.orientation.w = q_w
        pose.pose.covariance = [0.25, 0, 0, 0, 0, 0,
                                0, 0.25, 0, 0, 0, 0,
                                0, 0, 1e-9, 0, 0, 0,
                                0, 0, 0, 1e-9, 0, 0,
                                0, 0, 0, 0, 1e-9, 0,
                                0, 0, 0, 0, 0, 0.07]
        
        return pose
    
    def create_pose_stamped(self, position_x, position_y, orientation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        
        return pose
    

def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseClientNode()
    node.send_initial_pose(0.0, 0.0, 0.0) 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
    