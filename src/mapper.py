#!/usr/bin/python3

from turtle import distance
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
# from wall_following.srv import GetDistance, GetDistanceRequest

regions = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_dict = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

class SquareFollower():


    def __init__(self):
        
        rospy.init_node('wall_following_node', anonymous=False)
        self.state = 0
        
        self.dt = 0.005
        self.D = 0.6
        rate = 1/self.dt
        self.speed=0.1
        self.angular_speed=1
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    def get_distance_from_wall(self, dir):
        msg = rospy.wait_for_message("/scan" , LaserScan)
        ranges=np.array(msg.ranges)
        if len(ranges) == 0:
            return

        global regions
        regions = {
            'right':  min(min(ranges[245:289]), 10),
            'fright': min(min(ranges[290:336]), 10),
            'front':  min(min(ranges[:23]), min(ranges[337:]), 10),
            'fleft':  min(min(ranges[24:70]), 10),
            'left':   min(min(ranges[71:115]), 10),
        }

        self.check_route()

    def change_state(self, state):
        if state != self.state:
            self.state = state

    # def follow_wall(self):
    #     msg_twist = Twist()
    #     msg_twist.linear.x = 0.15
    #     return msg_twist

    def find_wall(self):
        msg_twist = Twist()
        msg_twist.linear.x = self.speed
        msg_twist.angular.z = 0
        self.speed=0.1
        return msg_twist
        
    def turn_left(self):
        msg_twist = Twist()
        msg_twist.angular.z =self.angular_speed
        self.angular_speed=0.5
        return msg_twist
    

    def turn_right(self):
        msg_twist = Twist()
        msg_twist.angular.z = -0.5
        return msg_twist

    def check_route(self):
        global regions
        rgns = regions
        
        state_description = ''
        d = self.D
        
        if rgns['front'] > d and rgns['fleft'] > d and rgns['fright'] > d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif rgns['front'] < d and rgns['fleft'] > d and rgns['fright'] > d:
            state_description = 'case 2 - front'
            ## random right or left
            self.change_state(random.randint(1,3))
        elif rgns['front'] > d and rgns['fleft'] > d and rgns['fright'] < d:
            state_description = 'case 3 - fright'
            self.change_state(0)
        elif rgns['front'] > d and rgns['fleft'] < d and rgns['fright'] > d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif rgns['front'] < d and rgns['fleft'] > d and rgns['fright'] < d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif rgns['front'] < d and rgns['fleft'] < d and rgns['fright'] > d:
            state_description = 'case 6 - front and fleft'
            self.change_state(2)
        elif rgns['front'] < d and rgns['fleft'] < d and rgns['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif rgns['front'] > d and rgns['fleft'] < d and rgns['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
            self.speed=0.4
        else:
            state_description = 'unknown case'
            rospy.loginfo(rgns)
        rospy.loginfo(state_description)

    def run(self):
        while not rospy.is_shutdown():
            self.get_distance_from_wall("right")
            print(self.state)
            if self.state == 0:
                cmd = self.find_wall()
            elif self.state == 1:
                cmd = self.turn_left()
            elif self.state == 2:
                cmd = self.turn_right()
            self.cmd_vel.publish(cmd)
            self.r.sleep()
try:
    pidc = SquareFollower()
    pidc.run()
except rospy.ROSInterruptException:
    rospy.loginfo("Navigation terminated.")