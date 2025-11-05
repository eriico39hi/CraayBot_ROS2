# CraayBot

Project to build a 2 wheeled robot using:
- Arduino for low level control
	- ISR to read encoders
	- MotorControl class for clean code
	- 2 pins for direction control
	- PWM for speed control

- ROS2 for high level control
	- Kinematics
    - Odometry
	- PID tuning
    - Movement programs
 
A serial bridge is used to communicate between the ROS PC and the Arduino.

See CraayBot_Arduino for Arduino code sections:
https://github.com/eriico39hi/CraayBot_Arduino
	
This is my CIS-600 - Masters Project for UMass Dartmouth
