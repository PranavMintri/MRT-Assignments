import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ProcessingNode(Node):
    def __init__(self):
        super().__init__('processing_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'unit_vector',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        # 1. Read the 2D vector
        x, y = msg.data[0], msg.data[1]
        self.get_logger().info(f'Received: x={x}, y={y}')

        # ---------------------------------------------------------
        # START PROCESSING PIPELINE (Your Code Here)
        # Goal: Transform (x, y) into a list of 18 values
        processed_values = [0.0] * 18 
        # END PROCESSING PIPELINE
        # ---------------------------------------------------------

        self.send_to_stm32(processed_values)

    def send_to_stm32(self, data_list):
        # This is where the communication logic (below) will live
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()