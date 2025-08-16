from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
import rclpy
from my_navbot_interfaces.msg import CurrentPose

class CurrentPoseNode(Node):
    def __init__(self):
        super().__init__('current_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_pose_pub_ = self.create_publisher(CurrentPose, 'current_pose', 10)
        self.timer_ = self.create_timer(1.0, self.get_current_pose)  # every second

    def get_current_pose(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map',         # target frame (global)
                'base_footprint',   # source frame (robot base)
                now
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q_x = trans.transform.rotation.x
            q_y = trans.transform.rotation.y
            q_z = trans.transform.rotation.z
            q_w = trans.transform.rotation.w
            e_x, e_y, e_z = tf_transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])
            self.call_current_pose_pub(x, y, e_z)
            self.get_logger().info(f"Robot pose: x={x:.2f}, y={y:.2f}, z={e_z:.2f}")
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("Transform from map to base_link not available yet.")


    def call_current_pose_pub(self, position_x, position_y, orientation_z):
        msg = CurrentPose()
        msg.x = position_x
        msg.y = position_y
        msg.theta = orientation_z

        self.current_pose_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CurrentPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()
