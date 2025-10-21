import rclpy
from rclpy.node import Node

# from std_msgs.msg import String


class TemplateNode(Node):

    def __init__(self):
        super().__init__('template_node')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)

def main(args=None):
    rclpy.init(args=args)

    node = TemplateNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
