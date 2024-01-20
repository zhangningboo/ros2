import rclpy
from rclpy.node import Node
from service_define_pkg.srv import AddTwoInts


class Server(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self.srv = self.create_service(srv_type=AddTwoInts, srv_name='add_two_ints', callback=self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Compute result: %d + %d = %d' % (request.a, request.b, response.sum))
        
        return response
    

def main(args=None):
    rclpy.init(args=args)
    server = Server('server_node')
    rclpy.spin(server)
    rclpy.shutdown()