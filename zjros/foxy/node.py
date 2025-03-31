import sys

import rclpy.node


class NodeBase(rclpy.node.Node):

    def __init__(self, name):
        rclpy.init(args=sys.argv)
        super().__init__(name)
        # log methods
        self.info = self.get_logger().info
        self.warn = self.get_logger().warn
        self.error = self.get_logger().error
        self.debug = self.get_logger().debug

    def __enter__(self):
        self.info(f"Successful initialization.")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass


# ros2 pkg create zjros2 --build-type ament_python --dependencies rclpy
def main():
    with NodeBase("demo") as node:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
