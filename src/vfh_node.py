#!/usr/bin/python3

from errno import EDEADLK, EROFS
import re
from rospkg import RosPack
import rospy
from nav_msgs.msg import Odometry
from math import radians,pi,degrees,atan2
from sensor_msgs.msg import LaserScan
import numpy as np
import tf
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time
import math
import csv


class VFHNode:
    
    def __init__(self) -> None:
        
        rospy.init_node("vfh_node" , anonymous=True)
        # self.target_x = rospy.get_param("/vfh_node/target_x")
        # self.target_y = rospy.get_param("/vfh_node/target_y")
        self.target_y = -8
        self.target_x = -8
        rospy.Subscriber("/scan",LaserScan,callback=self.turn_to_goal)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        # f= open('angles.csv', 'w')
        # self.writer = csv.writer(f)
        # self.angular_speed=0.1
        self.a=3.5
        self.b=1
        self.threshold=3
        self.smax=60
        self.yaw=0    
            
        self.k_theta=1.5
        self.k_i=0.6
        self.k_d=0.05
        self.k_i_max=50
        self.k_i_min=-50
        self.last_time= None
        self.sigma_error=0
        self.angular_error_last = 0.0
        self.vel= Twist()
        self.vel.linear.x = 0.25  # m/s
            

    def update_PID(self,angular_error, dt=None):

        if dt == None:
            cur_time = time.time()
            if self.last_time is None:
                self.last_time = cur_time 
            dt = cur_time - self.last_time
            self.last_time = cur_time
            
        if dt == 0 or math.isnan(dt) or math.isinf(dt):
            return 0.0

        p_term = self.k_theta * angular_error

        self.sigma_error += dt * angular_error

        i_term = self.k_i * self.sigma_error
        
        if i_term > self.k_i_max and self.k_i != 0:
            i_term = self.k_i_max
            self.sigma_error = i_term / self.k_i
        elif i_term < self.k_i_min and self.k_i != 0:
            i_term = self.k_i_min
            self.sigma_error = i_term / self.k_i

        self.d_error = (angular_error - self.angular_error_last) / dt
        self.angular_error_last = angular_error
        
        # Calculate derivative contribution to command 
        d_term = self.k_d * self.d_error
        rospy.loginfo(f"p: {p_term}, i: {i_term}, d:{d_term}")
        return p_term + i_term + d_term


    def angular_error(self, diff):
        if diff < -pi:
            return diff+ 2 * pi
        if diff > pi:
            return diff - 2 * pi
        return diff
        

    def find_goal(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        pose_x= msg.pose.pose.position.x
        pose_y= msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        self.yaw = yaw
        theta_star = atan2(self.target_y - pose_y, self.target_x - pose_x)
        if theta_star<0:
            theta_star+= 2*pi 
        if yaw<0:
            yaw+=2*pi
        goal_to_us=degrees(theta_star)-degrees(yaw)
        if goal_to_us<0:
            goal_to_us= goal_to_us+360
        
        # print(f"yaw {yaw}/{degrees(yaw)}, theta_star {theta_star}/{degrees(theta_star)} \n goal degree {goal_to_us}")

        return goal_to_us


    def get_valleys(self,signs):
        knear = 0
        ks = {}
        border_far=-1
        for i in range(0, len(signs) - 1):
            if signs[i] > 0 and signs[i + 1] < 0:
                knear=i
            if signs[i] < 0 and signs[i + 1] > 0:
                if knear==0:
                    border_far=i
                else:
                    ks[knear]=i

                    knear = 0
        if knear!=0:
            ks[knear]=border_far
        
        return ks

    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        # self.pose_x= msg.pose.pose.position.x
        # self.pose_y= msg.pose.pose.position.y
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        self.yaw = yaw
        return yaw
    

    def turn_to_goal(self,msg):

        # rospy.loginfo("\n\n")
        ranges=np.array(msg.ranges)          
        r=ranges[ranges<10000]
        m=self.a-self.b*ranges
        mp=self.a-self.b*r


        h=np.add.reduceat(m, np.arange(0, len(m), 5))
        hhh = np.hstack([h[-2:],h,h[:2]])
        smoothed = np.convolve([1,2,3,2,1], hhh)[4:-4]
        
        smoothed/=5
        smoothed_360=np.repeat(smoothed, 5)
        # rospy.loginfo(set(list(np.round(smoothed))))
        
        g = [self.threshold]*len(smoothed_360)
        signs = np.sign(smoothed_360 - g)
        goal_angle = int(self.find_goal())
        # rospy.loginfo(signs)
        rospy.loginfo(f"goal angle {goal_angle}")
        # ks = self.get_valleys(signs)
        edges=[i for i in range(0, len(signs) - 1) if signs[i] > 0 and signs[i + 1] < 0 or signs[i] < 0 and signs[i + 1] > 0]
        if len(edges)==1:
            return
        if signs[0]*signs[358]>0:
            edges=np.hstack([edges[-1],edges,edges[0]])
        else:
            edges=np.hstack([0,edges,358])
        
        rospy.loginfo(edges)
        closest=1000
        for i,e in enumerate(edges):
            min_dis=min(abs(e-goal_angle), 360- abs(e-goal_angle))
            if min_dis < closest:
                kstart=e
                closest=min_dis
                kstart_index=i
        # rospy.loginfo(f"kstart {kstart}, kstart_index{kstart_index}")
        zero=False
        if signs[kstart+1]<0:
            if kstart_index==len(edges)-1:
                kend=359
            else:
                kend=edges[kstart_index+1]
            rospy.loginfo(f"kstart {kstart}, kend {kend}")
            valley=kend-kstart
            if kend<kstart:
                valley=abs(360-kstart)+abs(kend-0)
                rospy.loginfo("zero included")
                zero=True
            if valley>self.smax:
                valley=self.smax
                rospy.loginfo("big valley")
            turn_angle=(kstart+valley/2)%360
            if goal_angle>kstart and goal_angle<kend:
                turn_angle=goal_angle+10
                rospy.loginfo("target in the valley")
            if zero and ((goal_angle>kstart and goal_angle<=360) or (goal_angle<kend and goal_angle>=0)):
                turn_angle=goal_angle+10
                rospy.loginfo("zero and target in the valley")
             
            
        else:
            kend=edges[kstart_index-1]
            rospy.loginfo(f"kstart {kstart}, kend {kend}")
        
            valley=kstart-kend
            if kend>kstart:
                valley=abs(360-kstart)+abs(kend-0)
                rospy.loginfo("zero included")
                zero=True
            if valley>self.smax:
                valley=self.smax
                rospy.loginfo("big valley")
            turn_angle=kstart-valley/2
            if turn_angle<0:
                turn_angle+=360
            if goal_angle<kstart and goal_angle>kend:
                turn_angle=goal_angle-10
                rospy.loginfo("target in the valley")
            if zero and ((goal_angle<kstart and goal_angle>=0 )or (goal_angle>kend and goal_angle<=360)):
                turn_angle=goal_angle-10
                rospy.loginfo("zero and target in the valley")

        rospy.loginfo(f"turn angle {turn_angle}")            


           
        # turn_angle= (turn_angle+90)%360
        if turn_angle>180:
            turn_angle= turn_angle-360
        # error=self.angular_error(radians(turn_angle)-self.get_heading())
        rospy.loginfo(f"obstacle distance{max(smoothed_360)}")
        error=turn_angle/180
        #*(9-max(smoothed_360))
        self.vel.angular.z = self.update_PID(error)
        # self.vel.angular.z= self.k_theta*
        rospy.loginfo(f"angular speed {self.vel.angular.z}\n\n")
        self.cmd_publisher.publish(self.vel)
        
 

if __name__ == "__main__":

    vfh = VFHNode()
    rospy.spin()