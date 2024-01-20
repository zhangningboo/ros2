import rclpy
from rclpy.node import Node
from service_define_pkg.srv import AddTwoInts


class Client(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self.cli = self.create_client(srv_type=AddTwoInts, srv_name='add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
    def send_request(self, a, b):
        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)

    def check_response(self):
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Result of add '+str(self.future.result().sum))
        else:
            self.get_logger().info('Exception caught: '+self.future.exception().__str__())


def main(args=None):
    rclpy.init(args=args)
    client = Client('client')
    client.send_request(3, 4)
    client.check_response()
    rclpy.shutdown()