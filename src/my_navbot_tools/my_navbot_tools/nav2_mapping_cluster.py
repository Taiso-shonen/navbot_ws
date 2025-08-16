#!/usr/bin/env python3
import time
import random
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sklearn.cluster import DBSCAN
from nav_msgs.msg import OccupancyGrid
from example_interfaces.msg import String
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from my_navbot_interfaces.srv import SetPose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA



class Nav2Mapping(Node): 
    def __init__(self):
        super().__init__("nav2_mapping")
        self.map_data_ = None
        self.is_goal_valid_ = None
        self.robot_cells_ = None
        self.current_goal_  = None
        self.frontier_cells_ = None
        self.clusters_ = None
        self.is_map_saved_ = False
        self.goal_history_ = set()
        self.map_subscriber_ = self.create_subscription(OccupancyGrid, '/map', self.callback_map, 10)
        self.goal_availability_sub_ = self.create_subscription(String, 'goal_availability', self.callback_goal_availability, 10)
        self.frontier_pub = self.create_publisher(Marker, '/frontier_markers', 10)
        self.unknown_pub = self.create_publisher(Marker, '/unknown_markers', 10)
        self.sending_goal_client_ = self.create_client(SetPose, 'send_goal')
        self.canceling_goal_client_ = self.create_client(Trigger, 'cancel_goal')
        self.map_saver_client_ = self.create_client(Trigger, 'save_map')


    def callback_map(self, msg: OccupancyGrid):
        self.map_data_ = msg
        # self.get_logger().info("New map recieved")
        map_array = self.get_map_array()

        self.frontier_cells_ = self.detect_frontier_cells(map_array)
        self.clusters_ = self.cluster_frontiers(self.frontier_cells_)
        self.get_logger().info(f"Detected {len(self.clusters_)} frontier clusters.")

        # self.publish_frontier_markers(clusters, 0.0, 0.0, 1.0, 1)
        # self.validate_goal(map_array)

        percentage = self.calculate_map_completion()
        if percentage >= 93.0 and not self.is_map_saved_:
            self.get_logger().info(f'Map completion percentage: {percentage}')
            self.call_map_saver_server()
            self.cancel_goal()
            self.is_map_saved_ = True


    def get_map_array(self):
        if self.map_data_ is not None:
            data = np.array(self.map_data_.data).reshape(self.map_data_.info.height, self.map_data_.info.width)
            return data
    

    def goal_generator(self):
        map_array = self.get_map_array()

        mixed_cells = np.argwhere(map_array != 100)

        # for cluster in frontier_clusters:
        
        x, y = self.pick_goal_from_clusters(self.clusters_, self.map_data_.info)
        self.robot_size_array(x, y)
        self.current_goal_ = (x, y)
        # self.is_goal_valid_, x_cells, y_cells = self.check_goal_cells(x, y, mixed_cells)
        # if self.is_goal_valid_:

        wx = self.map_data_.info.origin.position.x + x * self.map_data_.info.resolution
        wy = self.map_data_.info.origin.position.y + y * self.map_data_.info.resolution
        theta = round(random.uniform(-math.pi, math.pi), 2)
        if (x, y) not in self.goal_history_:
            self.goal_history_.add(self.current_goal_)

            # self.publish_frontier_markers(self.robot_cells_, 0.0, 0.0, 1.0, 0)
            return wx, wy, theta
    
        return None
    

    def callback_goal_availability(self, msg):
        # while self.map_subscriber_.get_publisher_count() == 0:
        #     self.get_logger().info("Waiting for a publisher to /map...")
        #     time.sleep(1)

        if self.map_data_ is not None and not self.is_map_saved_:
            self.get_logger().info("Sending a new goal")
            self.call_sending_goal_client()
        elif self.map_data_ is None:
            self.get_logger().warn("No map is being published")
        else:
            self.get_logger().info("Mapping is done. Map has been saved")


    def call_sending_goal_client(self):
        while not self.sending_goal_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server...")

        goal = self.pick_goal_from_clusters(self.clusters_, self.map_data_.info)
        if goal is None:
            self.get_logger().warn("Could not generate a valid goal")
            return
        x, y, theta = goal

        request = SetPose.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = self.sending_goal_client_.call_async(request)
        future.add_done_callback(self.callback_sending_goal_client)
        

    def callback_sending_goal_client(self, future):
        response: SetPose.Response = future.result()
        self.get_logger().info(response.msg)


    def robot_size_array(self, x, y, map_resolution=0.05, robot_size_x=0.4, robot_size_y=0.4, safety_margin=3):
        # Convert robot size to number of cells (+ safety)
        x_cells = int((robot_size_x / map_resolution)) + safety_margin * 2
        y_cells = int((robot_size_y / map_resolution)) + safety_margin * 2

        # Top-left corner of the robot's footprint
        start_y = int(y - y_cells // 2)
        start_x = int(x - x_cells // 2)

        self.robot_cells_ = []

        for i in range(y_cells):
            for j in range(x_cells):
                cell_y = start_y + i
                cell_x = start_x + j
                self.robot_cells_.append([cell_y, cell_x])  # row (y), column (x)

        # return x_cells, y_cells 
    

    def check_goal_cells(self, x, y, free_cells):
        self.robot_size_array(x, y)

        robot_cells_set = set(map(tuple, self.robot_cells_)) # the role of this line is to change each pare of coordinates into tuples in order to compare them as sets instead of numbers
        free_cells_set = set(map(tuple, free_cells))

        result = robot_cells_set.issubset(free_cells_set)
        return result
        
    
    # This runction returns a list of coordinates only of the cells adjacent to the robot_cells
    def outer_array(self, x, y, x_cells, y_cells):

        outer_cells = []

        outer_x_cells = x_cells + 2
        outer_y_cells = y_cells + 2
        outer_start_x = int(x - outer_x_cells // 2)
        outer_start_y = int(y - outer_y_cells // 2)
        
        for i in range(outer_y_cells):
            for j in range(outer_x_cells):
                if i == 0 or i == (outer_y_cells-1):
                    cell_y = outer_start_y + i
                    cell_x = outer_start_x + j
                    outer_cells.append([cell_y, cell_x])  # row (y), column (x)
            
                elif i > 0 and i < (outer_y_cells-1):
                    if j == 0 or j == (outer_x_cells-1):
                        cell_y = outer_start_y + i
                        cell_x = outer_start_x + j
                        outer_cells.append([cell_y, cell_x]) 

        return outer_cells
    
    def validate_goal(self, map_array):
        if self.robot_cells_ is not None and self.current_goal_ is not None:
            # self.publish_frontier_markers(self.robot_cells_, 0.0, 1.0, 0.0, 0)
            map_array = map_array
            mixed_cells = np.argwhere(map_array != 100)
            x = self.current_goal_[0]
            y = self.current_goal_[1]
            is_goal_valid = self.check_goal_cells(x, y, mixed_cells)
            if not is_goal_valid:
                self.get_logger().warn(f"Invalid goal detected at: ({x}, {y}). Cancelling.")
                self.cancel_goal()

    def calculate_map_completion(self):
        if self.map_data_ is None:
            return 0.0

        data = np.array(self.map_data_.data)
        total_cells = data.size
        known_cells = np.count_nonzero(data != -1)

        completion_percent = (known_cells / total_cells) * 100
        return completion_percent


    def call_map_saver_server(self):
        while not self.map_saver_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for map saver server...")

        request = Trigger.Request()
        
        future = self.map_saver_client_.call_async(request)
        future.add_done_callback(self.callback_call_map_saver_server)
        

    def callback_call_map_saver_server(self, future):
        response: Trigger.Response = future.result()
        self.get_logger().info(response.message)


    def detect_frontier_cells(self, map_array):
        frontiers = []
        rows, cols = map_array.shape

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] != 0:
                    continue  # not a free cell

                # Check 8-connected neighborhood
                neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                if -1 in neighbors:
                    frontiers.append((r, c))

        return frontiers


    def cluster_frontiers(self, frontier_cells, eps=3, min_samples=5):
        if (len(frontier_cells) == 0):
            return[]
        
        # Convert to numpy array
        points = np.array(frontier_cells)

        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)

        clusters = {}
        for i, label in enumerate(clustering.labels_):
            if label == -1:
                continue  # noise
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(frontier_cells[i])

        return list(clusters.values())
    

    def pick_goal_from_clusters(self, clusters, map_info):
        if not clusters:
            return None

        # Pick the largest cluster
        largest = max(clusters, key=len)
        
        # Convert cluster cells to (x, y) map coordinates
        goal_cells = [(
            map_info.origin.position.x + c[1] * map_info.resolution,
            map_info.origin.position.y + c[0] * map_info.resolution
        ) for c in largest]

        # Pick the center of the cluster (mean point)
        x = np.mean([pt[0] for pt in goal_cells])
        y = np.mean([pt[1] for pt in goal_cells])
        theta = round(random.uniform(-math.pi, math.pi), 2)

        # is_goal_valid = self.check_goal_cells(x, y, mixed_cells)
        # if is_goal_valid:
        return x, y, theta
        

    def publish_frontier_markers(self, robot_cells, r, g, b, x):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color = ColorRGBA(r=r, g=g, b=b, a=1.0)

        for cell in robot_cells:
            pt = Point()
            pt.x = self.map_data_.info.origin.position.x + cell[1] * self.map_data_.info.resolution
            pt.y = self.map_data_.info.origin.position.y + cell[0] * self.map_data_.info.resolution
            marker.points.append(pt)
        if x == 1:
            self.frontier_pub.publish(marker)
        else:
            self.unknown_pub.publish(marker)


    def cancel_goal(self):
        while not self.canceling_goal_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for cancel goal server...")

        request = Trigger.Request()
        
        future = self.canceling_goal_client_.call_async(request)
        future.add_done_callback(self.callback_cancel_goal)
        

    def callback_cancel_goal(self, future):
        response: Trigger.Response = future.result()
        if response.success:
            self.get_logger().info(response.message)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Mapping() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
