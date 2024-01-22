import rclpy
from rclpy.node import Node
from interface_define_pkg.srv import CustomService


class Server(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self.srv = self.create_service(srv_type=CustomService, srv_name='CustomService', callback=self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        response.result = True
        self.get_logger().info(f'Compute result: {request.name} {request.a} + {request.b} = {response.sum}')
        return response
    

def main(args=None):
    rclpy.init(args=args)
    server = Server('server_node')
    rclpy.spin(server)
    rclpy.shutdown()