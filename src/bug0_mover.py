#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

def poseCallback(pose_message):
    global x, y, yaw
    x= pose_message.pose.pose.position.x
    y= pose_message.pose.pose.position.y

    quaternion = (pose_message.pose.pose.orientation.x, pose_message.pose.pose.orientation.y, pose_message.pose.pose.orientation.z, 
                  pose_message.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    
    yaw = euler[2]


def scan_callback(scan_data):
    global dist_to_wall, ranges, range_len, range_increment
    ranges = scan_data.ranges
    range_len = len(ranges)
    range_increment = scan_data.angle_increment
    dist_to_wall = ranges[int(range_len /2)]#min(ranges[0:int(range_len/2)])#ranges[int(len(ranges)/2)]#ranges[next_angle_ind]


    
def rotate (velocity_publisher, angular_speed_degree, heading_degree_tol):
    #declare a Twist message to send velocity commands
    print("Finding the goal direction")
    velocity_message = Twist()

    heading_tol = math.radians(abs(heading_degree_tol))
    angular_speed = math.radians(abs(angular_speed_degree))
    velocity_message.angular.z = angular_speed

    loop_rate = rospy.Rate(2000)

    while True :
        heading_to_goal = math.atan2(y_goal-y, x_goal-x)
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        if  abs(yaw - heading_to_goal) < heading_tol:
            break

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)
    

def move(velocity_publisher, speed):
        #declare a Twist message to send velocity commands
        print("Heading towards goal")
        velocity_message = Twist()

        velocity_message.linear.x = speed  

        loop_rate = rospy.Rate(2000) # we publish the velocity at 100 Hz (10 times a second)    
        
        while True :
            velocity_publisher.publish(velocity_message)
            loop_rate.sleep()
            #print("x=", x, "   |   y= ", y, "   |   heading=", yaw*180/3.14, "[deg]   |   Distance to wall=", dist_to_wall)
            is_reached = abs(x - x_goal) < dist_to_goal_tol_x and abs(y - y_goal) < dist_to_goal_tol_y
            if  dist_to_wall < dist_to_wall_tol or is_reached or min(ranges) < dist_to_wall_tol / 1.2: 
                break
        velocity_message.linear.x =0
        velocity_publisher.publish(velocity_message)
        return is_reached

def parallel_to_wall (velocity_publisher, angular_speed_degree, heading_degree_tol):
    #declare a Twist message to send velocity commands
    print("Alligning parallel with the wall")
    velocity_message = Twist()

    # min_side_dist = min(ranges)
    min_ind = ranges.index(min(ranges))

    angular_speed = math.radians(abs(angular_speed_degree))
    velocity_message.angular.z = angular_speed / 2
    d_ind = min_ind - int(range_len /2)
    if  d_ind  > 0:
        dtheta = range_increment * d_ind + math.pi / 2
    else:
        dtheta = math.pi / 2 + range_increment * d_ind 


    check_ind = 181 #int(range_len /2) - int(math.pi / 2 / range_increment)
    
    freq = 1000
    loop_rate = rospy.Rate(freq)

    t0 = rospy.Time.now().to_sec()

    while True :        
        velocity_publisher.publish(velocity_message)
        t1 = rospy.Time.now().to_sec()
        rotated_angle = (t1-t0) * velocity_message.angular.z 
        loop_rate.sleep()
        if  rotated_angle >= dtheta: #abs(min_side_dist - ranges[check_ind])   < 0.01:
            break

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

def wall_following(velocity_publisher, speed):
        #declare a Twist message to send velocity commands
        print("Following the wall")
        velocity_message = Twist()

        min_side_dist = ranges[180]#min(ranges)

        velocity_message.linear.x = speed  

        loop_rate = rospy.Rate(1000) # we publish the velocity at 100 Hz (10 times a second)    
        
        while dist_to_wall > dist_to_wall_tol / 1.2:
            velocity_publisher.publish(velocity_message)
            loop_rate.sleep()
            if  abs(min_side_dist - ranges[180]) > 1 or dist_to_wall_tol * 3 < ranges[180]:
                dist_rot= 0
                velocity_message.angular.z = -math.pi / 5
                t10 = rospy.Time.now().to_sec()
                while abs(dist_rot) < math.pi / 3.6 and dist_to_wall > dist_to_wall_tol / 1.2:
                    #print("x=", x, "   |   y= ", y, "   |   heading=", yaw*180/3.14, "[deg]   |   Distance to wall=", dist_to_wall)
                    velocity_publisher.publish(velocity_message)
                    loop_rate.sleep()
                    t11 = rospy.Time.now().to_sec()
                    dist_rot = (t11-t10) * velocity_message.angular.z

                dist_trav = 0
                t00 = rospy.Time.now().to_sec()
                while dist_trav < 2 and dist_to_wall > dist_to_wall_tol / 1.2:
                    #print("x=", x, "   |   y= ", y, "   |   heading=", yaw*180/3.14, "[deg]   |   Distance to wall=", dist_to_wall)
                    velocity_publisher.publish(velocity_message)
                    loop_rate.sleep()
                    t01 = rospy.Time.now().to_sec()
                    dist_trav = (t01-t00) * velocity_message.linear.x
                break
        velocity_message.angular.z = 0
        velocity_message.linear.x = 0
        velocity_publisher.publish(velocity_message)
        dist_to_goal = ((x - x_goal) ** 2 + (y- y_goal) ** 2) ** 0.5
        angle_diff = yaw - math.atan2(y_goal-y, x_goal-x)
        diff_ind = int(range_len/2) - int(angle_diff / range_increment)
        if abs(angle_diff) < 3 * math.pi / 4 and ranges[diff_ind] > dist_to_wall_tol * 3 :
                isClear = True
        else:
            isClear = False
        return isClear

def final_orientation(velocity_publisher, angular_speed_degree, heading_degree_tol):
    #declare a Twist message to send velocity commands
    print("Rotationg to get to the desired final orientation")
    velocity_message = Twist()

    heading_tol = math.radians(abs(heading_degree_tol))
    angular_speed = math.radians(abs(angular_speed_degree))
    velocity_message.angular.z = angular_speed

    loop_rate = rospy.Rate(2000)

    while True :
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        if  abs(yaw - yaw_goal) < heading_tol:
            break

    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

if __name__ == '__main__':
    x_goal = 0
    y_goal = 20
    yaw_goal = 0

    linear_speed = 1.5
    angular_speed_degree = 25
    
    heading_degree_tol = 2
    dist_to_wall_tol = 1
    dist_to_goal_tol_x = 0.5
    dist_to_goal_tol_y = 0.5
    try:
        #initialize the node
        rospy.init_node('go_to_bug0', anonymous=True)

        #subscribe to the topic /scan. 
        scan_topic = "/base_scan"
        scan_subscriber = rospy.Subscriber(scan_topic, LaserScan, scan_callback)
        
        #subscribe to odoemetry
        position_topic = "/base_pose_ground_truth"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, poseCallback) 

        #declare velocity publisher
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        time.sleep(2)
        is_reached = False
        isClear = True


        while True:
            if isClear:
                rotate(velocity_publisher, angular_speed_degree, heading_degree_tol)
                is_reached = move(velocity_publisher, linear_speed)
            if is_reached:
                final_orientation(velocity_publisher, angular_speed_degree, heading_degree_tol)
                break
            parallel_to_wall (velocity_publisher, angular_speed_degree, heading_degree_tol)
            isClear = wall_following(velocity_publisher, linear_speed)
        print("Final Pose: (",x, ",", y,",",yaw*180/math.pi, " deg)") 
        print("Desired Pose: (",x_goal, ",", y_goal,",",yaw_goal*180/math.pi, " deg)") 
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
