import sys

import rclpy.node


class NodeBase(rclpy.node.Node):

    def __init__(self, name):
        super().__init__(name)
        # log methods
        self.info = self.get_logger().info
        self.warn = self.get_logger().warn
        self.error = self.get_logger().error
        self.debug = self.get_logger().debug


# ros2 pkg create zjros2 --build-type ament_python --dependencies rclpy
def main(name="demo", node_t=NodeBase):
    rclpy.init(args=sys.argv)
    node = node_t(name=name)
    node.info(f"Successful initialization.")
    rclpy.spin(node)
    rclpy.shutdown()
