#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
import tf_transformations
from example_interfaces.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from my_navbot_interfaces.srv import SetPose
from ament_index_python.packages import get_package_share_directory


def load_yaml_file(yaml_path):
    waypoints_path = yaml_path

    with open(waypoints_path, 'r') as f:
        waypoints = yaml.safe_load(f)

    return waypoints['waypoints']



class WaypointsPatrolNode(Node): 
    def __init__(self):
        super().__init__("waypoints_patrol")
        self.declare_parameter("yaml_path", value="")
        yaml_path = self.get_parameter("yaml_path").value
        self.waypoints_ = load_yaml_file(yaml_path)
        self.is_patrol_ = False
        self.counter_ = 0
        self.goal_response_sub_ = self.create_subscription(String, 'goal_response', self.callback_goal_response, 10) 
        self.goal_result_sub_ = self.create_subscription(String, 'goal_result', self.callback_goal_result, 10) 
        self.start_patrol_sevice_ = self.create_service(Trigger, "start_patrol", self.callback_start_patrol)
        self.stop_patrol_sevice_ = self.create_service(Trigger, "stop_patrol", self.callback_stop_patrol)
        self.cancel_patrol_client_ = self.create_client(Trigger, "cancel_goal")
        self.send_goal_client_ = self.create_client(SetPose, "send_goal")


    def call_send_goal(self):
        while not self.send_goal_client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for send goal service")

        if self.waypoints_ is None: 
            self.get_logger().info("No waypoints")
            return
        
        if self.counter_ >= len(self.waypoints_):
            self.get_logger().info("Patrol is finished")
            return
        
        if not self.is_patrol_:
            return

        goal = self.waypoints_[self.counter_]
        self.get_logger().info(f'Patrolling to position ({goal["x"]}, {goal["y"]}) and orientation {goal["yaw"]}')
        
        request = SetPose.Request()
        request.x = goal["x"]
        request.y = goal["y"]
        request.theta = goal["yaw"]
        future = self.send_goal_client_.call_async(request)
        future.add_done_callback(self.callback_send_goal)


    def callback_send_goal(self, future):
        try:
            response: SetPose.Response = future.result()
        except Exception as e:
            self.get_logger().error(f"Send goal service call failed: {e}")
            return


    def callback_goal_response(self, msg: String):
        if msg.data == "Goal got accepted":
            self.get_logger().info(msg.data)
        else:
            self.get_logger().warn(msg.data)


    def callback_goal_result(self, msg: String):
        if msg.data == "Succeeded":
            self.counter_ += 1
            self.get_logger().info(msg.data)
        elif msg.data == "Aborted":
            self.get_logger().warn(msg.data)
        elif msg.data == "Canceled":
            self.get_logger().info(msg.data)
        self.call_send_goal()
        

    def callback_start_patrol(self, request, response: Trigger.Response):
        if self.counter_ >= len(self.waypoints_):
            self.counter_ = 0
            
        self.is_patrol_ = True
        self.call_send_goal()

        response.success = True
        response.message = "Patrol has started"
        self.get_logger().info("Patrol has been started")
        return response
    

    def callback_stop_patrol(self, request,  response: Trigger.Response):
        self.is_patrol_ = False
        self.call_cancel_goal()
        response.success = True
        response.message = "Patrol has stopped"
        self.get_logger().info("Patrol has been stopped")
        return response
    
    
    def call_cancel_goal(self):
        request = Trigger.Request()
        self.cancel_patrol_client_.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsPatrolNode() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
