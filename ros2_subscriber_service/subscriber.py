import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts  # Service type

class SensorDataSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_data_subscriber')
        self.subscription = self.create_subscription(
            String, '/sensor_data', self.listener_callback, 10
        )
        self.client = self.create_client(AddTwoInts, '/process_data_service')

        self.get_logger().info('Waiting for service to be available...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service is now available.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        self.send_request(int(msg.data))

    def send_request(self, data):
        request = AddTwoInts.Request()
        request.a = data
        request.b = 10

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service Response: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
