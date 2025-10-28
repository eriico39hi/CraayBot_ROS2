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
WHEEL_DIA = 0.069 #m
WHEEL_R = WHEEL_DIA / 2
ROBOT_W = 0.2205 #m, center to center

#Kinematics class
class Kinematics(Node):
    def __init__(self):
        super().__init__('kinematics')
        
        #publish topics
        self.twistpub = self.create_publisher(Twist, 'robot_velocity', 10)
        
        #subscribe to topics
        self.jointsub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        
        self.get_logger().info("Kinematics node started")

    def inverse_kinematics(self,linearvel,angularvel):
        
    
        vleft_sp = (linearvel - (angularvel * ROBOT_W / 2.0))
        vright_sp = (linearvel + (angularvel * ROBOT_W / 2.0))
        
        wleft_sp = vleft_sp / WHEEL_R
        wright_sp = vright_sp / WHEEL_R
        
        return wleft_sp, wright_sp
    
    def forward_kinematics(self,wleft,wright):
        leftlinvel_rb = WHEEL_R * wleft
        rightlinvel_rb = WHEEL_R * wright
    
        linearv_rb = (leftlinvel_rb + rightlinvel_rb) / 2.0
        angularv_rb = (leftlinvel_rb - rightlinvel_rb) / ROBOT_W
        return linearv_rb, angularv_rb
        
    def joint_callback(self, msg: JointState):
    
        wleft_rb = msg.velocity[0]
        wright_rb = msg.velocity[1]
        
        v,w = self.forward_kinematics(wleft_rb, wright_rb)
        
        twistmsg = Twist()
        twistmsg.linear.x = v
        twistmsg.angular.z = w
        self.twistpub.publish(twistmsg)  
        
    

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
