#!/usr/bin/env python3
import cv2
import numpy as np
import math
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from my_navbot_interfaces.msg import ObjectTracker
from my_navbot_interfaces.srv import SetPose
import tf2_geometry_msgs
import tf2_ros
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class ObjectDetectorNode(Node): 
    def __init__(self):
        super().__init__("object_detector")
        self.depth_image_ = None
        self.is_camera_info_received_ = False
        self.detected_frames_ = 0
        self.bridge_ = CvBridge()
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        # Placeholder for intrinsics
        self.x_ = self.y_ = self.z_ = 0.0
        self.fx = self.fy = self.cx = self.cy = 0.0
        self.depth_camera_sub_ = self.create_subscription(Image, "/depth_camera", self.callback_depth_camera_sub, 10)
        self.rgb_camera_sub_ = self.create_subscription(Image, "/rgb_camera/image_raw", self.callback_rgb_camera_sub_, 10)
        self.camera_info_sub_ = self.create_subscription(CameraInfo, "/camera_info", self.callback_camera_info_sub, 10)
        self.msg_image_pub_ = self.create_publisher(Image, "/msg_image/image_raw", 10)
        self.object_position_pub_ = self.create_publisher(ObjectTracker, "object_pose", 10)
        self.send_goal_client_ = self.create_client(SetPose, "send_goal")
        # self.timer_ = self.create_timer(5.0, self.timer_callback)

    def callback_camera_info_sub(self, msg: CameraInfo ):
        self.fx = msg.k[0]  # fx
        self.fy = msg.k[4]  # fy
        self.cx = msg.k[2]  # cx
        self.cy = msg.k[5]  # cy
        self.is_camera_info_received_ = True

        # self.get_logger().info("Camera info received.")

        
    def callback_rgb_camera_sub_(self, msg: Image):
        if self.depth_image_ is None or not self.is_camera_info_received_:
            return
        
        rgb_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        x = msg.width // 2
        y = msg.height // 2
        r = int(60)

        # Color Detection "Red"
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # Define red color range in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for two red ranges (due to HSV wraparound)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Remove noise from mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours in mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        if contours:
            for contour in contours:
                min_area = 1000
                contour_area = cv2.contourArea(contour)

                if contour_area > min_area:
                    self.detected_frames_ += 1
                else:
                    self.detected_frames_ = 0

                if self.detected_frames_ >= 3:
                    # self.object_detected = True
                # Get the largest contour
                    # c = max(contours, key=cv2.contourArea)
                    # print(c)

                    # Draw rectangle around it
                    x, y, w, h = cv2.boundingRect(contour)
                    # cv2.rectangle(rgb_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                    # Draw contour shape
                    cv2.drawContours(rgb_image, [contour], -1, (255, 0, 0), 2)

                    # Draw center point
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(rgb_image, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(rgb_image, f"({cx}, {cy})", (cx+10, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

                        u, v = cx, cy
                        self.depth_calculation(u, v, x, y, r)        

                    # img = cv2.circle(rgb_image, (x, y), r, (0,0,255), 3)
                    msg_image = self.bridge_.cv2_to_imgmsg(rgb_image, encoding="passthrough")
                    self.msg_image_pub_.publish(msg_image)
                    return
        else:
            self.msg_image_pub_.publish(msg)

    def callback_depth_camera_sub(self, msg: Image):
        self.depth_image_ = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        

    def depth_calculation(self, u, v, x, y, r):
        depth_array = np.array(self.depth_image_, dtype=np.float32)
        if 0 <= v < depth_array.shape[0] and 0 <= u < depth_array.shape[1]:
            depth_value = depth_array[v, u]
            
            if np.isnan(depth_value) or depth_value <= 0:
                print(f"Invalid depth at (u={u}, v={v})")
                return
            

            if depth_array.dtype == np.uint16:
                depth_value = depth_value / 1000.0

            z = depth_value
            self.x_ = (u - self.cx) * (z / self.fx)
            self.y_ = (v - self.cy) * (z / self.fy)
            self.z_ = z
            if not z == math.inf:
                float_z = float(depth_value)
                self.publish_object_pose(u, v, x, y, r, float_z)

            # result = self.transform_point_to_map()
            # if result is not None:
            #     goal_x, goal_y = result
            #     print(f"3D Position: X={goal_x:.3f} m, Y={goal_y:.3f} m")

            # print(f"3D Position: X={self.x_:.2f} m, Y={self.y_:.2f} m, Z={self.z_:.2f} m")
            


    # def create_camera_pose(self):

    #     pose_camera = PoseStamped()
    #     pose_camera.header.frame_id = "camera_link"
    #     pose_camera.header.stamp = self.get_clock().now().to_msg()
    #     pose_camera.pose.position.x = self.x_
    #     pose_camera.pose.position.y = self.y_
    #     pose_camera.pose.position.z = self.z_
    #     pose_camera.pose.orientation.w = 1.0 # no rotation needed

    #     return pose_camera

    # def transformation_to_map_frame(self, pose_camera):
    #     try:
    #         pose_in_map = self.tf_buffer_.tr(
    #         "map",
    #         pose_camera.header.frame_id,
    #         rclpy.time.Time(),            # Time to get the transform
    #         timeout=Duration(seconds=1.0))
    #         goal_x = pose_in_map.pose.position.x
    #         goal_y = pose_in_map.transform.translation.y
    #         print((goal_x, goal_y))
    #         return goal_x, goal_y
    #     except (LookupException, ConnectivityException, ExtrapolationException) as e:
    #         self.get_logger().warn(f"Transform failed: {e}")
    #         return None
        

    def publish_object_pose(self, u, v, x, y, r, z):
        msg = ObjectTracker()
        msg.u = u
        msg.v = v
        msg.x = x
        msg.y = y
        msg.r = r
        msg.z = z
        self.object_position_pub_.publish(msg)


    def send_goal(self):
        while not self.send_goal_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server...")

        result = self.transform_point_to_map()
        if result is None:
            self.get_logger().warn("Transform to map frame failed. Skipping goal.")
            return
        goal_x, goal_y = result
        request = SetPose.Request()
        request.x = goal_x
        request.y = goal_y
        request.theta = 0.0

        future = self.send_goal_client_.call_async(request)
        future.add_done_callback(self.callback_send_goal)

    def callback_send_goal(self, future):
        response: SetPose.Response = future.result()
        self.get_logger().info(response.msg)

    def timer_callback(self):
        if self.depth_image_ is None or not self.is_camera_info_received_:
            return
        
        self.send_goal()
        self.timer_.cancel()



    def transform_point_to_map(self):
        # Create PointStamped in camera frame
        point_camera = PointStamped()
        point_camera.header.frame_id = "camera_link_optical"  # Your camera optical frame name
        point_camera.header.stamp = self.get_clock().now().to_msg()  # Use "now" instead of image stamp

        point_camera.point.x = self.x_
        point_camera.point.y = self.y_
        point_camera.point.z = self.z_

        try:
            # Lookup latest transform
            transform = self.tf_buffer_.lookup_transform(
                "map",                # Target frame
                point_camera.header.frame_id,  # Source frame
                rclpy.time.Time()     # Latest available time
            )

            # Transform the point
            transformed_point = tf2_geometry_msgs.do_transform_point(point_camera, transform)
            self.get_logger().info(f"Object in map frame: {transformed_point.point}")
            return transformed_point.point.x, transformed_point.point.y

        except Exception as e:
            self.get_logger().warn(f"Transform failed: {str(e)}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
