from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

# Import TFMessage
from tf2_msgs.msg import TFMessage

from nav_msgs.msg import Odometry

class StatePublisher(Node):

    def __init__(self):
        
        rclpy.init()
        super().__init__('epuck_state_publisher')

        qos_profile = QoSProfile(depth=10)
        # self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        self.timer = self.create_timer((1.0)/50, self.__publish_tf)

        # Create state publisher
        self.tf_publisher = self.create_publisher(TFMessage, '/tf', 1)

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'

        self.tf_msg =TFMessage()
        self.tf_msg.transforms.append(TransformStamped())
        self.tf_msg.transforms[0].header.frame_id = 'odom'
        self.tf_msg.transforms[0].child_frame_id = 'base_link'

        self.transform_init = False

        # OUR CODE
        self.gps_imu_odom_subscription = self.create_subscription(Odometry, '/gps_imu_odom', self.__process_gps_imu_odom, 1)

    def __process_gps_imu_odom(self, msg : Odometry):

        self.tf_msg.transforms[0].header.stamp = msg.header.stamp
        self.tf_msg.transforms[0].transform.translation.x = msg.pose.pose.position.x
        self.tf_msg.transforms[0].transform.translation.y = msg.pose.pose.position.y
        self.tf_msg.transforms[0].transform.translation.z = msg.pose.pose.position.z
        self.tf_msg.transforms[0].transform.rotation = msg.pose.pose.orientation

        self.transform_init = True

    def __publish_tf(self):

        if self.transform_init:
            try:
                self.tf_publisher.publish(self.tf_msg)
            except Exception as e:
                pass

def main():
    node = StatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()