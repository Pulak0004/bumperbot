import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")

        self.pub_ = self.create_publisher(String, "chatter", 10)

        self.frequency_ = 1.0
        self.get_logger().info("Publishing at %d Hz" %self.frequency_)

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_)

        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline()
            try:
                data.decode("utf-8")
            except:
                return
            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)

def main():
    rclpy.init()
    simple_serial_receiver = SimpleSerialReceiver()
    rclpy.spin(simple_serial_receiver)
    simple_serial_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()