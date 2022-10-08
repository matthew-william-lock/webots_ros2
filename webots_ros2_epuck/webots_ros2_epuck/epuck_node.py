# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 e-puck driver."""

from math import pi

from sympy import im
import rclpy
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan, Range, Imu
from geometry_msgs.msg import TransformStamped, PointStamped
from rclpy.node import Node
from functools import partial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from webots_ros2_epuck.quat import quat2euler,euler2quat


OUT_OF_RANGE = 0.0
INFRARED_MAX_RANGE = 0.04
INFRARED_MIN_RANGE = 0.009
TOF_MAX_RANGE = 1.0
NB_INFRARED_SENSORS = 8
SENSOR_DIST_FROM_CENTER = 0.035


DISTANCE_SENSOR_ANGLE = [
    -15 * pi / 180,   # ps0
    -45 * pi / 180,   # ps1
    -90 * pi / 180,   # ps2
    -150 * pi / 180,  # ps3
    150 * pi / 180,   # ps4
    90 * pi / 180,    # ps5
    45 * pi / 180,    # ps6
    15 * pi / 180,    # ps7
]


class EPuckNode(Node):
    def __init__(self):
        super().__init__('epuck_node')
        self.get_logger().info("Epuck node has been started.")

        # Intialize distance sensors for LaserScan topic
        self.__subscriber_dist_sensors = {}
        self.__distances = {}
        self.__tof_value = OUT_OF_RANGE
        for i in range(NB_INFRARED_SENSORS):
            self.__distances['ps{}'.format(i)] = OUT_OF_RANGE

        for i in range(NB_INFRARED_SENSORS):
            self.__subscriber_dist_sensors['ps{}'.format(i)] = \
                self.create_subscription(Range,
                                         '/ps{}'.format(i),
                                         partial(self.__on_distance_sensor_message, i),
                                         1)

        self.__subscriber_tof = self.create_subscription(Range, '/tof', self.__process_tof, 1)

        # Create odom topic based on GPS and IMU
        self.latest_GPS = None
        self.latest_IMU = None
        self.gps_subscription = self.create_subscription(PointStamped, '/gps', self.__process_gps, 1)
        self.gps_subscription = self.create_subscription(Imu, '/IMU', self.__process_IMU, 1)

        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 1)
        self.gps_odom_publisher = self.create_publisher(Odometry, '/gps_imu_odom', 1)

        self.__now = self.get_clock().now().to_msg()

        laser_transform = TransformStamped()
        laser_transform.header.stamp = self.__now
        laser_transform.header.frame_id = 'base_link'
        laser_transform.child_frame_id = 'laser_scanner'
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0
        laser_transform.transform.translation.x = 0.0
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.033

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(laser_transform)

        # Main loop self.get_clock
        # self.create_timer(50 / 1000, self.__publish_laserscan_data)
        self.__subscriber_tof = self.create_subscription(Odometry, '/odom', self.__publish_laserscan_data, 1)

        # Create a timer to update the odometry
        self.create_timer(1.0 / 5.0, self.__publish_odom)

    def __on_distance_sensor_message(self, i, msg):
        self.__distances['ps{}'.format(i)] = msg.range

        if i == 0:
            self.__now = msg.header.stamp

    def __process_tof(self, msg):
        self.__tof_value = msg.range

    def __process_gps(self, msg : PointStamped):

        # Save the latest GPS message
        self.latest_GPS = msg

    def __process_IMU(self, msg : Imu):

        # Save the latest IMU message
        self.latest_IMU = msg

    def __publish_odom(self):

        # Create the odometry message
        odom_msg = Odometry()

        # Set the header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # Set the child frame
        odom_msg.child_frame_id = 'base_link'

        # Set the position
        if self.latest_GPS is not None:

            # invert position
            odom_msg.pose.pose.position.x = -1*self.latest_GPS.point.x
            odom_msg.pose.pose.position.y = -1*self.latest_GPS.point.y
            odom_msg.pose.pose.position.z = self.latest_GPS.point.z

        # Set the orientation
        if self.latest_IMU is not None:

            # Rotate yaw by 180 degrees
            roll, pitch, yaw = quat2euler(self.latest_IMU.orientation.w, self.latest_IMU.orientation.x, self.latest_IMU.orientation.y, self.latest_IMU.orientation.z)
            yaw = yaw + pi
            qw, qx, qy, qz = euler2quat(0, 0, yaw)
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw

        # Publish the message
        self.gps_odom_publisher.publish(odom_msg)


    def __publish_laserscan_data(self, msg_odom):
        dists = [OUT_OF_RANGE] * NB_INFRARED_SENSORS
        dist_tof = OUT_OF_RANGE

        # Calculate distances
        for i, key in enumerate(self.__distances):
            dists[i] = self.__distances[key]

        # Publish range: ToF
        dist_tof = self.__tof_value

        # Max range of ToF sensor is 2m so we put it as maximum laser range.
        # Therefore, for all invalid ranges we put 0 so it get deleted by rviz
        laser_dists = [OUT_OF_RANGE if dist > INFRARED_MAX_RANGE else dist for dist in dists]
        dist_tof = OUT_OF_RANGE if dist_tof > TOF_MAX_RANGE else dist_tof
        msg = LaserScan()
        msg.header.frame_id = 'laser_scanner'
        msg.header.stamp = msg_odom.header.stamp
        msg.angle_min = - 150 * pi / 180
        msg.angle_max = 150 * pi / 180
        msg.angle_increment = 15 * pi / 180
        msg.range_min = SENSOR_DIST_FROM_CENTER + INFRARED_MIN_RANGE
        msg.range_max = SENSOR_DIST_FROM_CENTER + TOF_MAX_RANGE
        msg.ranges = [
            laser_dists[3] + SENSOR_DIST_FROM_CENTER,   # -150
            OUT_OF_RANGE,                               # -135
            OUT_OF_RANGE,                               # -120
            OUT_OF_RANGE,                               # -105
            laser_dists[2] + SENSOR_DIST_FROM_CENTER,   # -90
            OUT_OF_RANGE,                               # -75
            OUT_OF_RANGE,                               # -60
            laser_dists[1] + SENSOR_DIST_FROM_CENTER,   # -45
            OUT_OF_RANGE,                               # -30
            laser_dists[0] + SENSOR_DIST_FROM_CENTER,   # -15
            dist_tof + SENSOR_DIST_FROM_CENTER,         # 0
            laser_dists[7] + SENSOR_DIST_FROM_CENTER,   # 15
            OUT_OF_RANGE,                               # 30
            laser_dists[6] + SENSOR_DIST_FROM_CENTER,   # 45
            OUT_OF_RANGE,                               # 60
            OUT_OF_RANGE,                               # 75
            laser_dists[5] + SENSOR_DIST_FROM_CENTER,   # 90
            OUT_OF_RANGE,                               # 105
            OUT_OF_RANGE,                               # 120
            OUT_OF_RANGE,                               # 135
            laser_dists[4] + SENSOR_DIST_FROM_CENTER,   # 150
        ]
        self.laser_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = EPuckNode()
    rclpy.spin(epuck_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    epuck_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
