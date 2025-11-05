#This node will only convert wheel positions to robot pose
#Future expansion might invovle more expansive odometry information or a closed position loop

#Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import math

#Constants
WHEEL_DIA = 0.069 #m
WHEEL_R = WHEEL_DIA / 2
ROBOT_W = 0.2205 #m, center to center

#Odometry class
class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')
    
        #subscribe to topics
        self.jointsub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        
        #publish topics
        self.posepub = self.create_publisher(Pose, 'pose', 10)
        
        #init pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        #init prev pos for delta calcs
        self.prevleftpos = None
        self.prevrightpos = None
        
        self.get_logger().info("Odometry node started")
    
    def joint_callback(self, msg: JointState):
        leftpos = msg.position[0]
        rightpos = msg.position[1]
        
        #If first time, set prev pos and return
        if self.prevleftpos is None or self.prevrightpos is None:
            self.prevleftpos = leftpos
            self.prevrightpos = rightpos
            return
            
        #get deltas
        deltaleft = leftpos - self.prevleftpos
        deltaright = rightpos - self.prevrightpos
        
        deltalinleft = WHEEL_R * deltaleft
        deltalinright = WHEEL_R * deltaright
        
        deltapos = (deltalinleft + deltalinright) / 2.0
        deltatheta = (deltalinright - deltalinleft) / ROBOT_W
        
        #Update pose
        thetactr = self.theta + deltatheta / 2.0
        self.x += deltapos * math.cos(thetactr)
        self.y += deltapos * math.sin(thetactr)
        self.theta += deltatheta
        
        #Publish pose
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0
                
        theta = self.theta
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(self.theta / 2.0)
        pose.orientation.w = math.cos(self.theta / 2.0)
        
        self.posepub.publish(pose)
        
        #Update prev-wheel pos
        self.prevleftpos = leftpos
        self.prevrightpos = rightpos
        
#starts the node and keeps it alive with spin. kills it on ctrl+c
def main(args=None):
    rclpy.init(args=args)
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()