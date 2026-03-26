#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int8  # Added Int8 for the steering mode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# GLOBAL VARIABLES REMOVED! 
# We no longer use global vel_msg or mode_selection.

class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        self.wheel_seperation = 0.122*2
        self.wheel_base = 0.156*2
        self.wheel_radius = 0.026*2
        self.wheel_steering_y_offset = 0.03*2
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        self.pos = np.array([0,0,0,0], float)
        self.vel = np.array([0,0,0,0], float) 

        # NEW: Local variables to store incoming topic data
        self.current_vel = Twist()
        self.current_mode = 4 

        # NEW: Subscribers for the commands
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.mode_sub = self.create_subscription(Int8, '/steering_mode', self.mode_callback, 10)

        # Publishers to the hardware controllers (Unchanged)
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # NEW: Callback functions to update local variables when a message is received
    def cmd_callback(self, msg):
        self.current_vel = msg

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def timer_callback(self):
        # CHANGED: Replaced all global 'vel_msg' with 'self.current_vel'
        # CHANGED: Replaced 'mode_selection' with 'self.current_mode'

        # opposite phase
        if(self.current_mode == 1):
            vel_steerring_offset = self.current_vel.angular.z * self.wheel_steering_y_offset
            sign = np.sign(self.current_vel.linear.x)

            self.vel[0] = sign*math.hypot(self.current_vel.linear.x - self.current_vel.angular.z*self.steering_track/2, self.current_vel.angular.z*self.wheel_base/2) - vel_steerring_offset
            self.vel[1] = sign*math.hypot(self.current_vel.linear.x + self.current_vel.angular.z*self.steering_track/2, self.current_vel.angular.z*self.wheel_base/2) + vel_steerring_offset
            self.vel[2] = sign*math.hypot(self.current_vel.linear.x - self.current_vel.angular.z*self.steering_track/2, self.current_vel.angular.z*self.wheel_base/2) - vel_steerring_offset
            self.vel[3] = sign*math.hypot(self.current_vel.linear.x + self.current_vel.angular.z*self.steering_track/2, self.current_vel.angular.z*self.wheel_base/2) + vel_steerring_offset

            a0 = 2*self.current_vel.linear.x + self.current_vel.angular.z*self.steering_track
            a1 = 2*self.current_vel.linear.x - self.current_vel.angular.z*self.steering_track

            if a0 != 0:
                self.pos[0] = math.atan(self.current_vel.angular.z*self.wheel_base/(a0))
            else:
                self.pos[0] = 0
                
            if a1 != 0:
                self.pos[1] = math.atan(self.current_vel.angular.z*self.wheel_base/(a1))
            else:
                self.pos[1] = 0

            self.pos[2] = -self.pos[0]
            self.pos[3] = -self.pos[1]

        # in-phase
# in-phase
        elif(self.current_mode == 2):
            # math.atan2 handles all 4 quadrants perfectly without divide-by-zero errors
            ang = math.atan2(self.current_vel.linear.y, self.current_vel.linear.x)
            V = math.hypot(self.current_vel.linear.x, self.current_vel.linear.y)
            
            self.pos[0] = ang
            self.pos[1] = ang
            self.pos[2] = ang
            self.pos[3] = ang
            
            # Send the velocity to all wheels
            self.vel[:] = V
            
        # pivot turn
        elif(self.current_mode == 3):
            self.pos[0] = -math.atan(self.wheel_base/self.steering_track)
            self.pos[1] = math.atan(self.wheel_base/self.steering_track)
            self.pos[2] = math.atan(self.wheel_base/self.steering_track)
            self.pos[3] = -math.atan(self.wheel_base/self.steering_track)
            
            self.vel[0] = -self.current_vel.angular.z
            self.vel[1] = self.current_vel.angular.z
            self.vel[2] = self.vel[0]
            self.vel[3] = self.vel[1]

        else:
            self.pos[:] = 0
            self.vel[:] = 0

        pos_array = Float64MultiArray(data=self.pos) 
        vel_array = Float64MultiArray(data=self.vel) 
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)
        self.pos[:] = 0
        self.vel[:] = 0


class Joy_subscriber(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        
        # NEW: Publishers to send commands to the Commander node (or eventually, from AI)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(Int8, '/steering_mode', 10)

    def listener_callback(self, data):
        # NEW: Create local message objects to publish
        vel_msg = Twist()
        mode_msg = Int8()

        if(data.buttons[0] == 1):   
            mode_msg.data = 2
        elif(data.buttons[4] == 1): 
            mode_msg.data = 1
        elif(data.buttons[5] == 1): 
            mode_msg.data = 3
        else:
            mode_msg.data = 4

        vel_msg.linear.x = float(data.axes[1]*7.5)
        vel_msg.linear.y = float(data.axes[0]*7.5)
        vel_msg.angular.z = float(data.axes[3]*10.0)

        # NEW: Publish the messages to the ROS2 network
        self.cmd_pub.publish(vel_msg)
        self.mode_pub.publish(mode_msg)


if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()