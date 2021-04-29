#! /usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

class Environment():
    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odometry = Odometry()
        self.laser = LaserScan()
        self.position_robot_x = 0.
        self.position_robot_y = 0.
        rospy.sleep(1)
        
        
    def odometry_callback(self, data):
        self.odometry = data
        self.position_robot_x = data.pose.pose.position.x
        self.position_robot_y = data.pose.pose.position.y
    
    def laser_callback(self, data):
        self.laser = data
    
    def step(self, action):
        linear_vel = action[0]
        ang_vel = action[1]
        
        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel
        vel_cmd.angular.z = ang_vel
        
        self.pub_cmd_vel.publish(vel_cmd)
        
    def shutdown(self):
        rospy.loginfo("Lugar alcancado!!! o/ \o")
        self.pub_cmd_vel.publish(Twist())
        

if __name__ == "__main__":
    rospy.init_node('stage_controller_node', anonymous=False)
    
    env = Environment()
    
    target_x = -3.0
    target_y = 5.0
    
    min_distance = 0.3
    
    action = np.zeros(2)
    
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        x_robot = env.position_robot_x
        y_robot = env.position_robot_y
        
        distance_robot = math.sqrt((x_robot - target_x)**2 + (y_robot - target_y)**2)
        
        # Exemplo abaixo ----------------------------------------------------------------
        if distance_robot > min_distance:
            rospy.loginfo('Aonde estou: X--> %s, Y--> %s', x_robot, y_robot)
            
            if (min(env.laser.ranges) < 0.2):
                action[0] = 0.
                action[1] = -0.5
            else:
                action[0] = 0.3
                action[1] = 0.0
            
            env.step(action)
        else:
            env.shutdown()
        # Termino do exemplo--------------------------------------------------------------------------------
        
        # FAÇA O CÓDIGO AQUI
            
        r.sleep()