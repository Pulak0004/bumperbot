import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simple_service_server")
        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.service_callback)

        self.get_logger().info("Service for add two ints is ready!  ")

    def service_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info("Service is done. Returning sum = %d" % response.sum)
        return response

def main():
    rclpy.init()
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
