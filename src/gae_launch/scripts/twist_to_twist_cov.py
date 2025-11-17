import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped

class TwistToTwistCov(Node):
    def __init__(self):
        super().__init__('twist_to_twist_cov')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/vehicle/cmd_vel',
            self.twist_callback,
            10
        )
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            'output_twist_cov',
            10
        )
        self.get_logger().info('Twist to TwistWithCovariance converter started.')
    def twist_callback(self, msg):
        twist_cov_msg = TwistWithCovarianceStamped()
        twist_cov_msg.header.frame_id = "base_link"
        twist_cov_msg.header.stamp = self.get_clock().now().to_msg()
        twist_cov_msg.twist.twist = msg.twist
        twist_cov_msg.twist.covariance[0] = 0.5
        twist_cov_msg.twist.covariance[7] = 0.5
        twist_cov_msg.twist.covariance[14] = 0.5
        twist_cov_msg.twist.covariance[21] = 0.5
        twist_cov_msg.twist.covariance[28] = 0.5
        twist_cov_msg.twist.covariance[35] = 0.5
        self.publisher.publish(twist_cov_msg)
        # self.get_logger().info('Published TwistWithCovarianceStamped message.')

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistCov()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()