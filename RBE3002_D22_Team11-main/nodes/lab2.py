#!/usr/bin/env python2
from math import sqrt, pi, atan, asin , acos
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

def euclid_distance(start_x,start_y,current_x,current_y):
    return sqrt((current_x-start_x)**2 + (current_y - start_y)**2)

def get_angle_through(pos1_x,pos1_y,pos2_x,pos2_y):
    hyp = euclid_distance(pos1_x,pos1_x,pos2_x,pos2_y)
    x = pos2_x-pos1_x
    y = pos2_y-pos1_y 
    print(y/x)
    arc_cos = asin(x/hyp)
    arc_sin = asin(y/hyp)
    if arc_sin < 0:
        arccos = arc_cos*-1
    print(arc_cos)
    return arc_cos, hyp
    

def turn_left_or_right(curr_ang,look_up_angle):
    if curr_ang > 0:
        opposite = curr_ang - pi
        if look_up_angle < curr_ang and look_up_angle > opposite:
            return -1
        else :
            return 1
    if curr_ang < 0:
        opposite = curr_ang + pi
        if look_up_angle > curr_ang and look_up_angle < opposite:
            return 1
        else :
            return -1

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        self.px = 0
        self.py = 0
        self.pth = 0


        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        # TODO
        rospy.init_node("lab2")
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        # TODO
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        # TODO
        rospy.Subscriber("/odom", Odometry, self.update_odometry)
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        # TODO
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.go_to)

        rospy.sleep(5)
        



    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        new_speed = Twist() #create a message
        '''adding the veloxities'''
        new_speed.linear.x = linear_speed
        new_speed.angular.z = angular_speed
        ### Publish the message
        self.pub.publish(new_speed)
        

  

       
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        #stopping tolerance in meters
        stop_tol = .15
        ### REQUIRED CREDIT
        start_x = self.px
        start_y = self.py
        current_x = self.px
        current_y = self.py      
        
        if distance < 0:
            linear_speed = linear_speed * -1
        self.send_speed(linear_speed, 0)

        while euclid_distance(start_x,start_y,current_x,current_y) < abs(distance) - stop_tol:
            current_x,current_y = self.px,self.py
            rospy.sleep(.05)
        print("end drive")
        self.send_speed(0,0)

        


    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        print(angle)
        rotate_error_zone = 0.07

        
        
        

        #angle = angle % (2*pi)
        ### REQUIRED CREDIT
        
        
        if abs(angle) > pi:
            if angle >0:
                print(1)
                angle =  pi- angle
            else:
                print(2)
                angle = -pi + angle
        
        goal = angle + self.pth

        
        print(goal,self.pth)
        

        self.send_speed(0,aspeed)
        while not( self.pth <= goal+rotate_error_zone and self.pth >= goal-rotate_error_zone):
            rospy.sleep(.05)
        self.send_speed(0,0)
        


    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        #self.rotate(pi/2,-.5)
        ## REQUIRED CREDIT
        end_x = msg.pose.position.x
        end_y = msg.pose.position.y
        end_th = msg.pose.orientation.z

        desired_angle,distance = get_angle_through(self.px,self.py,end_x,end_y)
        print("desired angle",desired_angle)
        turn_angle  = desired_angle - self.pth
        self.rotate(turn_angle,.4)
        rospy.sleep(.5)
        self.drive(distance,.15)
        rospy.sleep(.5)

        turn_angle = end_th - self.pth

        turn_speed = .15
        if turn_angle < 0:
            turn_speed = turn_speed * -1
        self.rotate(turn_angle,turn_speed)
        

        
        
        
        



    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll , pitch , yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw  
        


    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        stop_tol = .005
        ### REQUIRED CREDIT
        start_x = self.px
        start_y = self.py
        current_x = self.px
        current_y = self.py      
        dist = euclid_distance(start_x,start_y,current_x,current_y)
        current_speed = 0
        maxspeed = .22
        if distance <0:
            linear_speed = linear_speed * -1
        self.send_speed(linear_speed, 0)
        while dist <distance - stop_tol:
            current_x,current_y = self.px,self.py
            distance = euclid_distance(start_x,start_y,current_x,current_y)
            speed_cap = .22
            print(euclid_distance(start_x,start_y,current_x,current_y))
            rospy.sleep(.05)
        self.send_speed(0,0)
        
        
        
        


    def run(self):
        print("start")
        """
        self.rotate(pi/2,.4)
        rospy.sleep(1)
        self.rotate(-pi/2,-.4)
        rospy.sleep(1)
        print("hi")
        print(3/2*pi)
        self.rotate(3*pi/2,.4)
        rospy.sleep(5)
        """
        rospy.spin()
        


if __name__ == '__main__':
    Lab2().run()
