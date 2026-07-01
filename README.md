# Line-Following-Robot-IR-Reflectance-Sensors

#### Summary: 
I built a ROS 2-based autonomous line-following robot from scratch using a Raspberry Pi and IR reflectance sensors. The robot performs real-time line detection using onboard sensors, generates steering commands through a rule-based control algorithm, and controls a differential-drive platform. 

#### Code Layout:
The demo video is associated with the code in the src folder.  The algorithm_improvements folder contains an implementation of a PID controller. This version demonstrates an alternative control approach, but the repository's demo video showcases the code in the src folder.

#### Video Link:
https://youtube.com/shorts/FZet1UYQgno

#### System Architecture:  

#### Hardware Used:   

#### Software Used:   
ROS 2 – Robotics middleware for inter-node communication.  
Ubuntu – Linux operating system running on the Raspberry Pi.  
C++ – Core robotics software implementation.  
libgpiod – Reads GPIO inputs from the IR reflectance sensors and controls motor direction pins.  
pigpio – Generates PWM signals for motor speed control.  

#### Algorithm and Implementation:

The robot uses a modular ROS 2 control pipeline to convert IR sensor readings into real-time motor commands for autonomous line following.

###### IR Sensor Processing
- Reads three IR reflectance sensors (left, center, and right) through the Raspberry Pi GPIO interface using `libgpiod`.
- Samples sensor values every 100 ms using a ROS 2 timer.
- Publishes velocity commands (`/cmd_vel`) based on the detected position of the line.

###### Rule-Based Line Following
- Uses a simple decision-based controller to determine robot motion.
- When the center sensor detects the line, the robot drives forward.
- When the left sensor detects the line, the robot commands a right turn.
- When the right sensor detects the line, the robot commands a left turn.
- If no sensor detects the line, the robot stops.

###### Differential Drive Motor Control
- Subscribes to `/cmd_vel` velocity commands.
- Converts linear and angular velocity commands into differential-drive motor commands.
- Controls motor direction using Raspberry Pi GPIO outputs connected to the L298N motor driver.

###### PWM Speed Control
- Uses the `pigpio` library to generate PWM signals for the left and right motors.
- Maintains a constant duty cycle during motion while allowing steering commands to independently control motor direction.

