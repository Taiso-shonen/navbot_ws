#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_navbot_interfaces.msg import ObjectTracker

class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__("object_tracker")
        
        self.last_detection_time = self.get_clock().now()
        self.object_detected = False
        self.current_linear_speed_ = 0.0
        self.last_obj_center_x_px = None
        self.last_image_center_x_px = None
        self.last_tracking_tolerance_px = None
        self.last_distance_m = None
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(ObjectTracker, "object_pose", self.callback_object_position, 10)
        
        self.create_timer(0.1, self.timer_check)  # safety check loop
        self.create_timer(0.1, self.robot_controller)

    def callback_object_position(self, msg: ObjectTracker):
        self.object_detected = True
        self.last_detection_time = self.get_clock().now()

        self.last_obj_center_x_px = msg.u
        self.last_image_center_x_px = msg.x
        self.last_tracking_tolerance_px = msg.r
        self.last_distance_m = msg.z



    def timer_check(self):
        # Stop robot if no detection for > 1 second
        if (self.get_clock().now() - self.last_detection_time).nanoseconds > 1e9:
            self.publish_cmd_vel(0.0, 0.0)
            self.object_detected = False

    def robot_controller(self):
        # if None in [self.last_obj_center_x_px, self.last_image_center_x_px, self.last_tracking_tolerance_px, self.last_distance_m]:
        #     angular = 0.2
        #     linear = 0.0

        if self.object_detected:
            k_angular = 0.002
            k_linear = 0.05

            error = self.last_obj_center_x_px - self.last_image_center_x_px
            angular = -k_angular * error

            if abs(error) < self.last_tracking_tolerance_px:
                angular *= 0.5
            
            if self.last_distance_m and 0.1 < self.last_distance_m < 5.0:
                target_linear = k_linear * (self.last_distance_m - 0.5)
                target_linear = max(0.0, min(0.3, target_linear))
            else:
                target_linear = 0.0
                # linear = min(1.0, max(0.0, (k_linear * (self.last_distance_m - 0.5))))
                # if linear == math.inf:
                #     self.get_logger().error("INF detected!")
                #     linear = 0.0
            linear = self.smooth_speed(target_linear)
        
        else:
            angular = -0.5
            linear = 0.0
        
        self.publish_cmd_vel(linear, angular)

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def smooth_speed(self, target, step=0.02):
        if target > self.current_linear_speed_:
            self.current_linear_speed_ += step
        elif target < self.current_linear_speed_:
            self.current_linear_speed_ -= step
        return self.current_linear_speed_

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
