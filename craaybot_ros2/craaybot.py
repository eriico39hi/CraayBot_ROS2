#Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import serial
import threading
import math

CTS_PER_REV = 984
RADS_PER_CT = (2 * math.pi) / CTS_PER_REV

#ROS2 <-> Arduino Serial Bridge
class Craaybot(Node):
    def __init__(self):
        super().__init__('craaybot_serial')

        #Serial setup - opens serial port @ 115200 baud
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info("Connected to Arduino!")

        #ROS setup - pubs encoder data, subscribes to "drive_command" and does callback upon receiving one
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(Int32MultiArray, 'speed', self.drive_callback, 10)

        #start constant serial read thread
        self.running = True
        threading.Thread(target=self.read_serial, daemon=True).start()

    #function for sending data to arduino
    #right now just f and s (forward and stop)
    def drive_callback(self, msg):
        leftSetPWM = msg.data[0]
        rightSetPWM = msg.data[1]
        if True:
            self.ser.write((str(leftSetPWM)+','+str(rightSetPWM)+'\n').encode())
            self.get_logger().info(f"Sent command: {leftSetPWM},{rightSetPWM}")
        else:
            self.get_logger().warn(f"Unknown command received: {msg.data}")

    #continuously reads encoder data and publishes it to encoder topic
    def read_serial(self):
        prevtime = self.get_clock().now().nanoseconds/1e9
        prevrpos = 0
        prevlpos = 0
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    #parse into single line
                    line = self.ser.readline().decode().strip()
                    
                    #once full line read, parse into string and publish to encoder topic
                    if line:
                        #Debug line for reading raw encoders in ROS2
                        #self.get_logger().info(f"Encoders:" + line)
                        
                        #Using JointState msg type for this, name set here, also get timestamp
                        curtime = self.get_clock().now().nanoseconds/1e9
                        msg = JointState()
                        msg.name = ['left_wheel', 'right_wheel']
                        
                        #convert string to 2 numbers, then convert counts to rad, store in 'position'
                        leftenc,rightenc = [int(x) for x in line.split(',')]
                        leftpos = leftenc * RADS_PER_CT
                        rightpos = rightenc * RADS_PER_CT
                        msg.position = [leftpos, rightpos]
                        
                        #calculate velocity and store into velocity as rad/s
                        deltalpos = leftpos - prevlpos
                        deltarpos = rightpos - prevrpos
                        deltatime = curtime - prevtime
                        
                        if (deltatime > 0.0):
                            leftvel = deltalpos/deltatime
                            rightvel = deltarpos/deltatime
                        else:
                            leftvel = 0.0
                            rightvel = 0.0
                        msg.velocity = [leftvel, rightvel]
                        
                        #set prev values
                        prevtime = curtime
                        prevlpos = leftpos
                        prevrpos = rightpos

                        #publish message to other nodes
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
