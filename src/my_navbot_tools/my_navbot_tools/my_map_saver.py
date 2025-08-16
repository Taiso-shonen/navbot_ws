#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import SaveMap
from std_srvs.srv import Trigger


class MyMapSaverNode(Node): 
    def __init__(self):
        super().__init__("my_map_saver") 
        self.declare_parameter("map_name", "my_map")
        self.declare_parameter("image_format", "pgm")
        self.declare_parameter("map_mode", "trinary")
        self.declare_parameter("free_thresh", 0.25)
        self.declare_parameter("occupied_thresh", 0.65)
        self.map_name_ = self.get_parameter("map_name").value
        self.image_format_ = self.get_parameter("image_format").value
        self.map_mode_ = self.get_parameter("map_mode").value
        self.free_thresh_ = self.get_parameter("free_thresh").value
        self.occupied_thresh_ = self.get_parameter("occupied_thresh").value
        self.map_url_ = f'/home/ts/ros2_ws/src/my_navbot_bringup/maps/{self.map_name_}'
        self.map_ = None
        self.map_subscriber_ = self.create_subscription(OccupancyGrid, '/map', self.map_subscriber_callback, 10)
        self.map_saver_server_ = self.create_service(Trigger, 'save_map', self.map_saver_server_callback)
        self.nav2_save_map_client_ = self.create_client(SaveMap, '/map_saver/save_map')

        
    def map_subscriber_callback(self, msg: OccupancyGrid):
        self.map_ = msg

    def map_saver_server_callback(self, request, response: Trigger.Response):
        if self.map_ is None:
            response.success = False
            response.message = "No map data recieved"
            return response
        self.call_nav2_map_saver_server()
        # if result.success:
        response.success = True
        response.message = "Map saved to my_navbot_bringup/maps"
        self.get_logger().info('Map saved to my_navbot_bringup/maps')
        return response
        # else:
        #     response.success = False
        #     response.message = "Nav2 map saver server isn't available"
        #     return response

    
    def call_nav2_map_saver_server(self):
        while not self.nav2_save_map_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for map saver server...")

        request = SaveMap.Request()
        request.map_topic = 'map'
        request.map_url = self.map_url_
        request.image_format = self.image_format_
        request.map_mode = self.map_mode_
        request.free_thresh = self.free_thresh_
        request.occupied_thresh = self.occupied_thresh_

        future = self.nav2_save_map_client_.call_async(request)
    #     result = future.add_done_callback(self.callback_call_nav2_map_saver_server)
    #     return result
        
    # def callback_call_nav2_map_saver_server(self, future):
    #     response = future.result()
    #     return response.result


def main(args=None):
    rclpy.init(args=args)
    node = MyMapSaverNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
