import rclpy
import rclpy.node
import custom_msgs.msg

import random as rand

class Talker(rclpy.node.Node):
    def __init__(self, node_name, node_ns, start_parameter_services = False, enable_rosout=False):
        super().__init__(node_name, namespace=node_ns, start_parameter_services=start_parameter_services, enable_rosout=enable_rosout)
        self.declare_parameter('target_topic', rclpy.Parameter.Type.STRING)
        target_topic = self.get_parameter_or('target_topic', self.get_fully_qualified_name() + 'info').value
        self.get_logger().info(f'Got target topic: {target_topic}')
        self.pub = self.create_publisher(custom_msgs.msg.Pose2DStamped, target_topic, 10)
        self.timer = self.create_timer(2, self.timer_cb)
        self.get_logger().info(f'Init of node: "{self.get_fully_qualified_name()}" successfull')

    def send_message(self, x, y, theta):
        self.get_logger().info(f'Got Message {x=:05.2f}, {y=:05.2f}, {theta=:06.2f}')
        msg = custom_msgs.msg.Pose2DStamped()
        msg.pose.x = x
        msg.pose.y = y
        msg.pose.theta = theta
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def timer_cb(self):
        self.send_message(rand.uniform(0, 10), rand.uniform(1, 10), rand.uniform(0, 360))

def main(args=None):
    rclpy.init(args=args)
    talker_node = Talker('talker_node', '/INTP_ROS', False, False)
    try:
        rclpy.spin(talker_node)
    finally:
        talker_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()