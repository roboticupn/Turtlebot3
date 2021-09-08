#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf import transformations


import math
import numpy as np

#robot state
_position = Point ()
_yaw = 0

#move state
_state = 0
activate = False
#path
goal_position = Point()
goal_position.x = 0
goal_position.y = 1
goal_position.z = 0

#parameters
yaw_precision = math.pi / 90 # +/-
dist_precision = 0.2

kp_distance = 1
kd_distance = 0.5

kp_angle = 1
kd_angle = 0.05


linear_speed = 1
angular_speed = 1
#publishers
pub = None 
pub_empty = None

def odom_callback(msg):
    global _position, _yaw

    #position
    _position = msg.pose.pose.position
    print msg.pose.pose


    #yaw
    quaternion=(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    # euler = [roll,pitch,yaw]
    _yaw = euler[2]

def change_state(state):
    global _state

    _state = state
    print 'Status State: [%s]'% _state #

def finish():
    move = Twist()
    move.linear.x = 0
    move.angular.z = 0
    
    pub.publish(move)
    pub_empty.publish(Empty())
    rospy.logout("Finish")

def go_ahead(_pos):
    global _yaw, pub, yaw_precision, _state
    prev_dist= 0
    prev_angle=0
    last_rotation = 0
    goal_angle = math.atan2(_pos.y - _position.y, _pos.x - _position.x)
    error_angle = goal_angle - _yaw
    error_pos = math.sqrt (pow(_pos.y - _position.y,2) + pow(_pos.x - _position.x,2)) #theta
    move = Twist()
    if error_pos > dist_precision:
        if goal_angle < -math.pi/4 or goal_angle > math.pi/4:
            if _pos.y <0 and _position.y < _pos.y:
                goal_angle = -2*math.pi + goal_angle
            elif _pos.y >= 0 and _position > _pos.y:
                goal_angle = 2*math.pi + goal_angle
        if last_rotation > math.pi-0.1 and _yaw <= 0:
            _yaw = 2*math.pi + _yaw
        elif last_rotation < -math.pi+0.1 and _yaw > 0:
            _yaw = -2 * math.pi + _yaw
        diff_angle = goal_angle - prev_angle
        diff_dist = error_pos - prev_dist

        distance = math.sqrt (pow(_pos.y - _position.y,2) + pow(_pos.x - _position.x,2))
        #pd
        adjust_distance = kp_distance*distance + kd_distance*diff_dist
        adjust_angle = kp_angle*goal_angle + kd_angle*diff_angle
        
        move.angular.z = adjust_angle -_yaw
        move.linear.x = min(adjust_distance, 0.1)

        if move.angular.z > 0:
            move.angular.z = min (move.angular.z, 1.5)
        else:
            move.angular.z = max (move.angular.z, -1.5)
        if error_angle > math.pi:
            error_angle = error_angle -((2*math.pi*error_angle)/error_angle)

        last_rotation = _yaw
        pub.publish(move)
        prev_dist = distance
    
    else:
        print 'Error Position: [%s]' % error_pos
        change_state(2)

    if math.fabs(error_angle) > yaw_precision:
        print 'Error Angle: [%s]' % error_pos
        change_state(0)


def fix_heading(_pos):
    global _yaw, pub, yaw_precision, _state
    goal_angle = math.atan2(_pos.y - _position.y, _pos.x - _position.x)
    error_angle = goal_angle - _yaw

    if error_angle > math.pi:
        error_angle = error_angle -((2*math.pi*error_angle)/error_angle)

    move = Twist()
    if math.fabs(error_angle) > yaw_precision:
        if error_angle > 0:
            move.angular.z = 0.3
            move.linear.x = 0  
            pub.publish(move)
        else:
            move.angular.z = -0.3
            move.linear.x = 0  
            pub.publish(move)

    if math.fabs(error_angle)<=yaw_precision:
        print 'Error Angle: [%s]'% error_angle
        change_state(1)



def main():
    global pub,_state, pub_empty

    rospy.init_node('goto_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_empty = rospy.Publisher('mobile_base/command/reset_odometry', Empty, queue_size=10)
    sub = rospy.Subscriber('/odom', Odometry, odom_callback)

   

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        
        if _state == 0:
            fix_heading(goal_position)
        elif _state == 1:
            go_ahead(goal_position)
        elif _state == 2:
            finish()
            pass
        else:
            rospy.logerr('Unknown State!')
            pass
        rate.sleep()



if __name__ == "__main__":
    main()