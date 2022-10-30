import rclpy
import rclpy.node
import std_msgs.msg

import random as rand

class Talker(rclpy.node.Node):
    def __init__(self, node_name, node_ns):
        super().__init__(node_name, namespace=node_ns)
        self.declare_parameter('target_topic', rclpy.Parameter.Type.STRING)
        target_topic = self.get_parameter_or('target_topic', self.get_fully_qualified_name() + 'info').value
        self.get_logger().info(f'Got target topic: {target_topic}')
        self.pub = self.create_publisher(std_msgs.msg.String, target_topic, 10)
        self.timer = self.create_timer(2, self.timer_cb)
        self.get_logger().info(f'Init of node: "{self.get_fully_qualified_name()}" successfull')

    def send_message(self, str):
        self.get_logger().info(f'Got Message: "{str}"')
        msg = std_msgs.msg.String()
        msg.data = str
        self.pub.publish(msg)

    def timer_cb(self):
        self.send_message("Got magic number: {:03d}".format(rand.randrange(200)))

def main(args=None):
    rclpy.init(args=args)
    talker_node = Talker('talker_node', '/INTP_ROS')
    try:
        rclpy.spin(talker_node)
    finally:
        talker_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()