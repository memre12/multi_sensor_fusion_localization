#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
from nav_msgs.msg import Odometry


class RetimerNode(Node):
    def __init__(self):
        super().__init__('retimer_node')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/sbg_device2/imu/velocity',
            self.callback,
            10
        )

        self.subscription_imu = self.create_subscription(
            Imu,
            'sbg_device2/imu/data',
            self.callback_imu,
            10
        )

        self.subscription_lidar = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.callback_lidar,
            10
        )

        self.subscription_odom = self.create_subscription(
            Odometry,
            'sbg/ros/odometry',
            self.callback_odom,
            10
        )

        self.publisher_odom = self.create_publisher(
            Odometry,
            '/sbg/ros/odometry_corrected',
            10
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            '/sbg/ros/twist_with_covariance_stamped_corrected',
            10
        )
        self.publisher_imu = self.create_publisher(
            Imu,
            '/sbg/ros/imu/data_corrected_',
            10
        )

        self.subscription_gnss = self.create_subscription(
            NavSatFix,
            '/imu/nav_sat_fix',
            self.callback_gnss,
            10
        )

        self.subscription_gnss2 = self.create_subscription(
            NavSatFix,
            '/sbg_device2/imu/nav_sat_fix',
            self.callback_gnss2,
            10
        )

        self.publisher_gnss = self.create_publisher(
            NavSatFix,
            '/imu/nav_sat_fix_corrected',
            10
        )

        self.publisher_gnss2 = self.create_publisher(
            NavSatFix,
            '/sbg_device2/imu/nav_sat_fix_corrected',
            10
        )

        self.subscription_gnss_orientation = self.create_subscription(
            GnssInsOrientationStamped,
            '/autoware/sbg/ins',
            self.callback_gnss_orientation,
            10
        )
        self.publisher_gnss_orientation = self.create_publisher(
            GnssInsOrientationStamped,
            '/sbg/ros/gnss_ins_orientation_corrected',
            10
        )
        self.publisher_lidar = self.create_publisher(
            PointCloud2,
            'velodyne_points_corrected',
            10
        )
        self.get_logger().info("Retimer node başlatıldı.")

    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link2'
        self.publisher.publish(msg)

    def callback_imu(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link2'
        self.publisher_imu.publish(msg)

    def callback_gnss(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss_link'
        self.publisher_gnss.publish(msg)

    def callback_gnss_orientation(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_gnss_orientation.publish(msg)

    def callback_odom(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link_corrected'
        msg.child_frame_id = 'velodyne_corrected'
        self.publisher_odom.publish(msg)

    def callback_lidar(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'velodyne_corrected'
        self.publisher_lidar.publish(msg)
    
    def callback_gnss2(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gnss_link2'
        self.publisher_gnss2.publish(msg)

        

def main(args=None):
    rclpy.init(args=args)
    node = RetimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
