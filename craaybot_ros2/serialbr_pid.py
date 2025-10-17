#Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult
import serial
import threading
import math

#Constants
CTS_PER_REV = 984
RADS_PER_CT = (2 * math.pi) / CTS_PER_REV

MAX_INT = 100

PORT_NAME = '/dev/ttyACM0'

#PID tuning class
class PIDController:
    def __init__(self,kp,ki,kd,maxint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.maxint = maxint
        
        self.prevtime = None
        self.preverror = 0.0
        self.integral = 0
        
        
    def compute(self, setpoint, measured, curtime):
        error = setpoint - measured
        
        if self.prevtime is None:
            dtime = 0.02  #initial deltatime
        else:
            dtime = curtime - self.prevtime
            
        #Apply proportional gain term
        pterm = self.kp * error
        
        #Apply integral gain term
        self.integral += error * dtime
        self.integral = max(-self.maxint, min(self.maxint,self.integral)) #stop integral windup
        iterm = self.ki * self.integral
        
        #Apply derivitive gain term
        if self.prevtime is not None:
            dterm = self.kd * (error - self.preverror)/dtime
        else:
            dterm = 0
            
        #Compute full calculation with all 3 terms
        output = pterm + iterm + dterm
        
        #State updates
        self.preverror = error
        self.prevtime = curtime
        
        return int(output)

#ROS2 <-> Arduino Serial Bridge Class
class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
       
        #Serial setup - opens serial port @ 115200 baud
        self.ser = serial.Serial(PORT_NAME, 115200, timeout=1)
        self.get_logger().info("Connected to Arduino!")

        
        #PID tuning vals as ROS parameters
        self.declare_parameter('kp',20)
        self.declare_parameter('ki',0.1)
        self.declare_parameter('kd',5.0)
        
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        
        self.get_logger().info(f"PID Gains: Kp = {kp}, Ki = {ki}, Kd = {kd}")

        #ROS setup - pubs encoder data, subscribes to "drive_command" and does callback upon receiving one
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'speed', self.speed_callback, 10)
        
        #PID controllers for each wheel (same gains for now)
        self.leftpid = PIDController(kp,ki,kd,MAX_INT)
        self.rightpid = PIDController(kp,ki,kd,MAX_INT)
        
        #Allow live parameter updates
        self.add_on_set_parameters_callback(self.on_param_change)
        
        #init speeds
        self.leftsetspeed = 0.0
        self.rightsetspeed = 0.0
        
        #init velocities
        self.leftspeed = 0.0
        self.rightspeed = 0.0
        
        #control loop
        self.controlrate = 20.0
        self.controltimer = self.create_timer(1.0/self.controlrate, self.control_loop)

        #start constant serial read thread
        self.running = True
        threading.Thread(target=self.read_serial, daemon=True).start()

    def on_param_change(self, params):
        
        for param in params:
            if param.name == 'kp':
                self.leftpid.kp = param.value
                self.rightpid.kp = param.value
            elif param.name == 'ki':
                self.leftpid.ki = param.value
                self.rightpid.ki = param.value
            elif param.name == 'kd':
                self.leftpid.kd = param.value
                self.rightpid.kd = param.value
                
        kp = self.leftpid.kp
        ki = self.leftpid.ki
        kd = self.leftpid.kd
        self.get_logger().info(f"PID Gains: Kp = {kp}, Ki = {ki}, Kd = {kd}")

        return SetParametersResult(successful = True)

    #function for sending data to arduino
    #right now just f and s (forward and stop)
    def speed_callback(self, msg):
        self.leftsetspeed = msg.data[0]
        self.rightsetspeed = msg.data[1]
        
        self.get_logger().info(f"Updated Speed: {self.leftsetspeed}rad/s,{self.rightsetspeed}rad/s")
            
    def control_loop(self):
        curtime = self.get_clock().now().nanoseconds / 1e9
        
        #PID doesn't behave nice at 0 speed, tries to ramp down but it cant go super slow...
        #skip past this and just set 0PWM if desired speed = 0
        if (self.leftsetspeed == 0):
            leftSetPWM = 0
        else:
            if(self.leftsetspeed < 7):
                self.leftsetspeed = 7
                self.get_logger().info("Can't set speed below 7, clamping up to 7")
            if(self.leftsetspeed > 12):
                self.leftsetspeed = 12
                self.get_logger().info("Can't set speed above 12, clamping down to 12")
            leftSetPWM = self.leftpid.compute(self.leftsetspeed, self.leftspeed, curtime)
            
        if (self.rightsetspeed == 0):
            rightSetPWM = 0
        else:
            if(self.rightsetspeed < 7):
                self.rightsetspeed = 7
                self.get_logger().info("Can't set speed below 7, clamping up to 7")
            if(self.rightsetspeed > 12):
                self.rightsetspeed = 12
                self.get_logger().info("Can't set speed above 12, clamping down to 12")
            rightSetPWM = self.rightpid.compute(self.rightsetspeed, self.rightspeed, curtime)
        
        try:
            self.ser.write((str(leftSetPWM)+','+str(rightSetPWM)+'\n').encode())
            #debug to check PWM values send, fills log pretty hard
            #self.get_logger().info(f"PWM SetPoints: {leftSetPWM}, {rightSetPWM}")
            
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

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
                        self.leftspeed = leftvel
                        self.rightspeed = rightvel
                        
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
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
