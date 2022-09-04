#!/usr/bin/python3

import re
import networkx as nx
# import matplotlib.pyplot as plt


# from dis import dis
import rospy
import tf

# from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from math import radians

import numpy as np
import math
import time


def read_pgm(filename, byteorder='>'):

    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

def get_graph(image):
    hx, hy = np.where(image[1:] & image[:-1]) #horizontal edge start positions
    h_units = np.array([hx, hy]).T
    h_starts = [tuple(n) for n in h_units]
    h_ends = [tuple(n) for n in h_units + (1, 0)] #end positions = start positions shifted by vector (1,0)
    horizontal_edges = zip(h_starts, h_ends)

    #CONSTRUCTION OF VERTICAL EDGES
    vx, vy = np.where(image[:,1:] & image[:,:-1]) #vertical edge start positions
    v_units = np.array([vx, vy]).T
    v_starts = [tuple(n) for n in v_units]
    v_ends = [tuple(n) for n in v_units + (0, 1)] #end positions = start positions shifted by vector (0,1)
    vertical_edges = zip(v_starts, v_ends)

    G = nx.Graph()
    G.add_edges_from(horizontal_edges)
    G.add_edges_from(vertical_edges)
    # pos = dict(zip(G.nodes(), G.nodes())) # map node names to coordinates
    # nx.draw_networkx(G, pos, with_labels=False, node_size=0)
    # labels={node: f'({node[0]},{node[1]})' for node in G.nodes()}
    # nx.draw_networkx_labels(G, pos, labels, font_size=6, font_family='serif', font_weight='bold', bbox = dict(fc='lightblue', ec="black", boxstyle="round", lw=1))
    # plt.show()
    return G

def get_point(x,y,funky_coords,l):
  for i in range(l):
    for j in range(l):
      if funky_coords[i][j][0]==x and funky_coords[i][j][1]==y:
        return (i,j)


def generate_path():
    image = read_pgm("/home/pariya/Documents/robotics/catkin_hw2/src/vfh/src/map.pgm", byteorder='<')
    image_copied=np.copy(image)
    image_copied[image_copied<250]=1
    image_copied[image_copied>250]=0
    origin= [-22.800000, -21.200000, 0.000000]

    mapper_p=[]
    l=image.shape[0]

    for j in range(l,0,-1):
        y0=round(origin[1]+0.05*(l-j),2)
        line=[]
        for i in range(l):
            line.append((y0,round(origin[0]+0.05*i,2)))
        mapper_p.append(line)
    funky_coords=np.array(mapper_p[::-1])

    G=get_graph(1-image_copied)
    goal=get_point(8,8,funky_coords,l)
    source_node=get_point(-8.5,-8.5,funky_coords,l)
    # colors={node: "gray" for node in G.nodes() }

    p1 = nx.shortest_path(G, source=source_node, weight='weight')
    # for p in p1[goal]:
    #   colors[p]="red"

    path_x=[]
    path_y=[]
    for (i,j) in p1[goal]:
        path_x.append(funky_coords[i][j][1])
        path_y.append(funky_coords[i][j][0])

    return (np.array(path_x[::-1]), np.array(path_y[::-1]))

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("shortest_path" , anonymous=False)
        
        # self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        # self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        # self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(90) # rad
        # self.stop_distance = rospy.get_param("/rectangle/stop_distance") # m
        # self.epsilon = rospy.get_param("/rectangle/epsilon")
        self.width = 2
        self.length = 3
        self.pose_x = 0
        self.pose_y = 0
        self.vel= Twist()
        self.vel.linear.x = 0.2  # m/s
        self.vel.angular.z = 0.1  # rad/s
        # defining the states of our robot
        self.GOw, self.GOl, self.ROTATEl, self.ROTATEw = 0, 1, 2, 3
        self.state = self.GOw
        self.p=3
        self.i_error=8
        self.counter = 8
        self.ds=0.1
        self.k_p=0.2 #0.2
        self.k_theta=1.3 #1.4
        self.k_i=0.0001
        self.k_d=0.005
        self.k_i_max=50
        self.k_i_min=-50
        self.last_time= None
        self.sigma_error=0
        self.distance_error_last = 0.0
        self.path=generate_path()
    
    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        self.pose_x= msg.pose.pose.position.x
        self.pose_y= msg.pose.pose.position.y
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        self.yaw = yaw
        return yaw
    
    def angular_error(self, goal_x, goal_y):
        theta_star = math.atan2(goal_y - self.pose_y, goal_x - self.pose_x)
        if theta_star - self.yaw < -math.pi:
            return theta_star- self.yaw + 2 * math.pi
        if theta_star - self.yaw > math.pi:
            return theta_star- self.yaw - 2 * math.pi
        rospy.loginfo(f"theta_star:{theta_star}, yaw: {self.yaw}")
        return theta_star - self.yaw

    def dist(self, goal_x, goal_y):
        return math.sqrt(((goal_x - self.pose_x) ** 2) + ((goal_y - self.pose_y) ** 2))- self.ds

    def update_PID(self,distance_error, dt=None):

        if dt == None:
            cur_time = time.time()
            if self.last_time is None:
                self.last_time = cur_time 
            dt = cur_time - self.last_time
            self.last_time = cur_time
            
        if dt == 0 or math.isnan(dt) or math.isinf(dt):
            return 0.0

        p_term = self.k_p * distance_error

        self.sigma_error += dt * distance_error

        i_term = self.k_i * self.sigma_error
        
        if i_term > self.k_i_max and self.k_i != 0:
            i_term = self.k_i_max
            self.sigma_error = i_term / self.k_i
        elif i_term < self.k_i_min and self.k_i != 0:
            i_term = self.k_i_min
            self.sigma_error = i_term / self.k_i

        self.d_error = (distance_error - self.distance_error_last) / dt
        self.distance_error_last = distance_error
        
        # Calculate derivative contribution to command 
        d_term = self.k_d * self.d_error
        rospy.loginfo(f"p: {p_term}, i: {i_term}, d:{d_term}")
        return p_term + i_term + d_term

    def run(self):
        tmp=0
        sigma_error=0
        path=self.path
        while not rospy.is_shutdown():
            rospy.loginfo("\n\n\n")
               
            self.get_heading()

            goal_x, goal_y = path[0][tmp],path[1][tmp]

            distance_error = self.dist(goal_x, goal_y)
            linear_speed= self.update_PID(distance_error)
            
            # last_sigma_error = sigma_error
            # sigma_errpor= distance_error

            if distance_error < self.ds:
                rospy.loginfo(f"step {tmp}")
                tmp += 10
                self.i_error = 0


            if tmp == len(path)-1:
                tmp = 1
                self.counter += 1

            angle_to_goal = self.angular_error(goal_x, goal_y)
            
          
 
            # angle_to_goal = angle_to_goal - self.yaw    
            
            # if abs(angle_to_goal) > 0.05:
            z_counterclock = self.k_theta * angle_to_goal
            
            

            rospy.loginfo(f"Angular_error = {angle_to_goal}")
            rospy.loginfo(f"GOAL X, Y {goal_x}, {goal_y}")
            self.vel.linear.x = linear_speed
            self.vel.angular.z = z_counterclock
            
            rospy.loginfo(f"SELF = {self.pose_x},{self.pose_y}")
            rospy.loginfo(f"LOCATION Follower {tmp}")
            rospy.loginfo(f"DISTANCE = {distance_error}" )
      

            self.cmd_publisher.publish(self.vel)
            now = rospy.get_rostime()
            # rospy.loginfo("Time now: ", now.secs)
            next = 0.3
            rospy.loginfo(f"Twist: {self.vel.linear.x}, {self.vel.angular.z}")
            
            rospy.sleep(next)
            

if __name__ == "__main__":
    controller = Controller()
    controller.run()