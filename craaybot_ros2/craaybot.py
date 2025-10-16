#Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

#ROS2 <-> Arduino Serial Bridge
class Craaybot(Node):
    def __init__(self):
        super().__init__('craaybot_serial')

        #Serial setup - opens serial port @ 115200 baud
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info("Connected to Arduino on /dev/ttyACM0")

        #ROS setup - pubs encoder data, subscribes to "drive_command" and does callback upon receiving one
        self.pub = self.create_publisher(String, 'encoders', 10)
        self.sub = self.create_subscription(String, 'drive_command', self.drive_callback, 10)

        #start constant serial read thread
        self.running = True
        threading.Thread(target=self.read_serial, daemon=True).start()

    #function for sending data to arduino
    #right now just f and s (forward and stop)
    def drive_callback(self, msg):
        command = msg.data.strip().lower()
        if command in ['f', 's']:
            self.ser.write(command.encode())
            self.get_logger().info(f"Sent command: {command}")
        else:
            self.get_logger().warn(f"Unknown command received: {msg.data}")

    #continuously reads encoder data and publishes it to encoder topic
    def read_serial(self):
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    #parse into single line
                    line = self.ser.readline().decode().strip()
                    #once full line read, parse into string and publish to encoder topic
                    if line:
                        msg = String()
                        msg.data = line
                        self.pub.publish(msg)
            #exception for when serial port goes dark
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")

    #stops cleanly if ROS shuts down (standard practice)
    def destroy_node(self):
        self.running = False
        self.ser.close()
        super().destroy_node()

#starts the node and keeps it alive with spin. kills it on ctrl+c
def main(args=None):
    rclpy.init(args=args)
    node = Craaybot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
