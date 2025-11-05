
#Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import math

class Move(Node):
    def __init__(self):
        super().__init__('move')
        
        #init x and y at 0
        self.startx = None
        self.starty = None
        
        #Publishers and subscribers
        self.cmdpub = self.create_publisher(Twist, 'robot_velocity_sp', 10)
        self.posesub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        
        #Velocity setpoint and variable for finished
        self.velsp = 0.28
        self.finished = False

    def pose_callback(self, msg: Pose):
    
        #dont do anything if we already moved
        if self.finished:
            self.create_timer(0.5, self.shutdown_node)
            return
            
        x = msg.position.x
        y = msg.position.y
        
        #if first iter, set this as startpos and exit
        if self.startx is None:
            self.startx = x
            self.starty = y
            return
        
        dx = x - self.startx
        dy = y - self.starty
        distance = math.sqrt(dx*dx + dy*dy)
        
        twist = Twist()
        
        if distance < 1.0:
            twist.linear.x = self.velsp
            twist.angular.z = 0.0
            
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.finished = True
            self.get_logger().info("Reached 1 meter!")
            
        self.cmdpub.publish(twist)
        
    def shutdown_node(self):
        self.get_logger().info("Stopping move node, movement complete!")
        self.destroy_node()
        rclpy.shutdown()
        
#starts the node and keeps it alive with spin. kills it on ctrl+c
def main(args=None):
    rclpy.init(args=args)
    node = Move()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()