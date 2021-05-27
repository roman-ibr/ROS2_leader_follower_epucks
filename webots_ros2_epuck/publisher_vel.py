#######################################################################################
# Name and Surname: Roman Ibrahimov
# Email address: ibrahir@purdue.edu
#
# Program Description: The objective of this code is to simulate a pair of mobile robots,
# in which a slave robot follows the leader. The simulation is based on Webots robot simulator 
# which is integrated with ROS2. Each robot has its own controller. The positions of the robots
# in 3D space is published on /odom topic simultaneously.Based on the information received from /odom,
# the leader robot estimates its heading direction and translational velocity to reach the waypoints. 
# In parallel, the slave robot is following the leader. 
#
########################################################################################


import rclpy
import math
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped 
from math import sin, cos, atan2
import numpy as np
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from builtin_interfaces.msg import Time



class Publisher_vel(Node):
    def __init__(self):
        # node initialization
        super().__init__('cmd_publihser_vel')
        # x and y odometry data for agent 1
        self.odom_x =0.0
        self.odom_y =0.0
        # x and y odometry data for agent 2
        self.odom_x_2 =0.0
        self.odom_y_2 =0.0
        #distance to the goal for each agent 
        self.rho = float("inf")
        self.rho_2 = float("inf")
        # pitch angles for each agent 
        self.yaw=0
        self.yaw_2=0
        # variable that keeps the track of waypoints 
        self.loop = 0

        # odometry subscriber and velocity publisher for each aent 
        self.sub = self.create_subscription(Odometry, 'odom_1', self.callback, 10) 
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_2 = self.create_subscription(Odometry, 'odom_2', self.callback_2, 10) 
        self.publisher_2 = self.create_publisher(Twist, 'cmd_vel_2', 10)

        # frequency for the iteration 
        timer_period = 0.0001
        # timer 
        self.timer=self.create_timer(timer_period, self.timer_callback)

    #callback function for each agent to get sensor data 
    def callback(self, msg):
        # reading x and y translation of the agent 
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        # converting quaternion to Euler angle 
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        self.yaw = math.atan2(t3, t4)

    def callback_2(self, msg):

        self.odom_x_2 = msg.pose.pose.position.x 
        self.odom_y_2 = msg.pose.pose.position.y - 0.2

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        self.yaw_2 = math.atan2(t3, t4)

        # function that publishes velocity commands for each robot 
    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        # we cannot publish higher translational velocity than given threshold
        max_speed = 1
        # if the given velocity is higher than the threshold, we should merge it down
        abs_v = abs(linear_velocity)
        if (abs_v < max_speed):
            vx = linear_velocity
            wz = angular_velocity
        else:
            vx = linear_velocity / abs_v * max_speed
            wz = angular_velocity / abs_v * max_speed
        # velocity message type is Twist 
        # we are publishing velocity commands as twist message
        msg_2 = Twist()
        msg_2.linear.x = vx
        msg_2.angular.z = wz
        self.publisher.publish(msg_2) 

    def move_2(self, linear_velocity=0.0, angular_velocity=0.0):
        max_speed = 1
        abs_v = abs(linear_velocity)
        if (abs_v < max_speed):
            vx = linear_velocity
            wz = angular_velocity
        else:
            vx = linear_velocity / abs_v * max_speed
            wz = angular_velocity / abs_v * max_speed
        msg_2 = Twist()
        msg_2.linear.x = vx
        msg_2.angular.z = wz
        self.publisher_2.publish(msg_2) 


    # controller to lead the robot to the target 
    # t_x and t_y are the agent's target potition, od_x and od_y are the agent's current coorditanes 
    # od_yaw is the agent's current pitch angle 
    def to_go(self, t_x=0.0, t_y=0.0, od_x=0.0, od_y=0.0, od_yaw=0.0):

        # controller tunes 
        k_rho = 0.1
        k_alpha = 0.9
        k_beta = -0.23

        # calculating target angle
        dx = t_x - od_x
        dy = t_y - od_y
        target_rad = math.atan2(dy, dx)

        # calculating the distance between two agents 
        dis_x = np.abs(self.odom_x - self.odom_x_2)
        dis_y = np.abs(self.odom_y - self.odom_y_2)
        dist = math.sqrt(dis_x**2 + dis_y**2) 

        # the robot should apprach to the target if the distance is bigger than the threshold
        if self.rho>=0.15:
            self.rho = math.sqrt(dx**2 + dy**2) 
            alpha = normalize(math.atan2(dy, dx) - od_yaw)
            beta = normalize(target_rad - math.atan2(dy, dx))
            v = k_rho * self.rho
            w = k_alpha * alpha + k_beta * beta
            # if the distance between the two robots are smaller than threshold, the leader should move
            # otherwise, it will wait for the slave robot
            if(dist < 0.19):
                self.move(v, w)
            else: 
                self.move(0.0, 0.0)
        # when the agent reaches the goal, we set a new waypoint
        elif self.rho<0.15:
            self.rho = float("inf")
            self.move(0.0, 0.0)
            self.loop =self.loop+1
            # the robots continuesly looping around the 5 waypoints 
            if(self.loop ==5):
                self.loop = 0

        else: 
            self.move(0.0, 0.0)



    def to_go_2(self, t_x=0.0, t_y=0.0, od_x=0.0, od_y=0.0, od_yaw=0.0):

        k_rho = 0.1
        k_alpha = 0.9
        k_beta = -0.23

        dx = t_x - od_x
        dy = t_y - od_y
        target_rad = math.atan2(dy, dx)


        dis_x = np.abs(self.odom_x - self.odom_x_2)
        dis_y = np.abs(self.odom_y - self.odom_y_2)
        dist = math.sqrt(dis_x**2 + dis_y**2) 


        if self.rho_2>=0.05:
            self.rho_2 = math.sqrt(dx**2 + dy**2)  
            alpha = normalize(math.atan2(dy, dx) - od_yaw)
            beta = normalize(target_rad - math.atan2(dy, dx))
            v = k_rho * self.rho_2
            w = k_alpha * alpha + k_beta * beta
            # if the robots are too close to each other, the slave robot should wait until the master gets apart
            if(dist > 0.1):
                self.move_2(v, w)
            else:
                self.move_2(0.0, 0.0)
        elif self.rho_2<0.05:
            self.rho_2 = float("inf")
            self.move_2(0.0, 0.0)
        else: 
            self.move_2(0.0, 0.0)


    
    # this function continuesly repeat the loop 
    def timer_callback(self):

        # here we have five waypoints, each waypoint is enumerated 
        if (self.loop ==0):
            print("Heading to waypoint 1")
            # the leader robot moves to the given waypoint, whereas the slave one follows it
            self.to_go(0.3, 0.6, self.odom_x, self.odom_y, self.yaw)
            self.to_go_2(self.odom_x, self.odom_y, self.odom_x_2, self.odom_y_2, self.yaw_2)

        elif (self.loop==1):
            print("Heading to waypoint 2")
            self.to_go(0.6, 0.0, self.odom_x, self.odom_y, self.yaw)
            self.to_go_2(self.odom_x, self.odom_y, self.odom_x_2, self.odom_y_2, self.yaw_2)

        elif (self.loop==2):
            print("Heading to waypoint 3")
            self.to_go( 0.0, -0.6, self.odom_x, self.odom_y, self.yaw)
            self.to_go_2(self.odom_x, self.odom_y, self.odom_x_2, self.odom_y_2, self.yaw_2)

        elif (self.loop==3):
            print("Heading to waypoint 4")
            self.to_go( -0.6, -0.2, self.odom_x, self.odom_y, self.yaw)
            self.to_go_2(self.odom_x, self.odom_y, self.odom_x_2, self.odom_y_2, self.yaw_2)

        elif (self.loop==4):
            print("Heading to waypoint 5")
            self.to_go( 0.0, 0.0, self.odom_x, self.odom_y, self.yaw)
            self.to_go_2(self.odom_x, self.odom_y, self.odom_x_2, self.odom_y_2, self.yaw_2)


def main(args=None):
    # initializing ROS2
    rclpy.init(args=args)
    # creating Publisher_vel object 
    publisher_object = Publisher_vel()
    # it keeps the loop running
    rclpy.spin(publisher_object)
    # we should not kep the node after we are done 
    publisher_object.destroy_node()
    rclpy.shutdown()
        

if __name__ == '__main__':

    main()


