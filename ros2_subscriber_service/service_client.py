import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class DataProcessingService(Node):
    def __init__(self):
        super().__init__('data_processing_service')
        self.srv = self.create_service(AddTwoInts, '/process_data_service', self.process_request)
        self.get_logger().info('Service is ready to process requests.')

    def process_request(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Processing request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessingService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
