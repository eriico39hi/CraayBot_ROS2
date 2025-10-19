#This node will input set linear and angular velocities, convert to wheel rad/s and send to
#serialbr/PID which will convert to PWM% for each wheel (forward kinematic)
#reads back JointStates (enc pos in rad, enc vel in rad/s) converts this to Twist (inverse kinematic)

#Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

#Constants
WHEEL_DIA = 69 #mm
ROBOT_W = 220.5 #mm, center to center

#Kinematics class
class Kinematics(Node):
    def __init__(self):
        super().__init__('kinematics')
        self.get_logger().info("Kinematics node started")       

    #stops cleanly if ROS shuts down (standard practice)
    def destroy_node(self):
        super().destroy_node()

#starts the node and keeps it alive with spin. kills it on ctrl+c
def main(args=None):
    rclpy.init(args=args)
    node = Kinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
