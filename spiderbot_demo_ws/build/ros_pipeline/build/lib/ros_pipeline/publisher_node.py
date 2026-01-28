import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class UnitVectorPublisher(Node):
    def __init__(self):
        super().__init__('unit_vector_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'unit_vector', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32MultiArray()
        # Your 2D vector: [x, y]
        msg.data = [1.0, 0.5] 
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Unit Vector: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = UnitVectorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()