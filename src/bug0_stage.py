#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class bug0:

    def __init__(self, x_goal, y_goal, yaw_goal, linear_speed, angular_speed_degree, heading_degree_tol, dist_to_wall_tol, dist_to_goal_tol_x, dist_to_goal_tol_y):
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.yaw_goal = yaw_goal
        self.linear_speed = linear_speed
        self.angular_speed_degree = angular_speed_degree
        self.heading_degree_tol = heading_degree_tol
        self.dist_to_wall_tol = dist_to_wall_tol
        self.dist_to_goal_tol_x = dist_to_goal_tol_x
        self.dist_to_goal_tol_y = dist_to_goal_tol_y


        self.heading_tol = math.radians(abs(self.heading_degree_tol))
        self.angular_speed = math.radians(abs(self.angular_speed_degree))

        #initialize the node
        rospy.init_node('bug0_alg', anonymous=True)

        #subscribe to odoemetry
        position_topic = "/base_pose_ground_truth"
        self.pose_subscriber = rospy.Subscriber(position_topic, Odometry, self.poseCallback) 

        #subscribe to the topic /scan. 
        scan_topic = "/base_scan"
        self.scan_subscriber = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)

        #declare velocity publisher
        cmd_vel_topic='/cmd_vel'
        self.velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.velocity_message = Twist()
        
        time.sleep(2)
        self.freq = 2000

    def poseCallback(self, pose_message):
        self.x = pose_message.pose.pose.position.x
        self.y = pose_message.pose.pose.position.y

        quaternion = (pose_message.pose.pose.orientation.x, pose_message.pose.pose.orientation.y, pose_message.pose.pose.orientation.z, 
                    pose_message.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        
        self.yaw = euler[2]

    def scan_callback(self, scan_data):
        self.ranges = scan_data.ranges
        self.range_len = len(self.ranges)
        self.range_increment = scan_data.angle_increment
        self.dist_to_wall = self.ranges[int(self.range_len /2)]
        
    def rotate (self):
        print("Finding the goal direction")
        self.velocity_message.angular.z = self.angular_speed
        loop_rate = rospy.Rate(self.freq)

        while True :
            heading_to_goal = math.atan2(self.y_goal-self.y, self.x_goal-self.x)
            self.velocity_publisher.publish(self.velocity_message)
            loop_rate.sleep()
            if  abs(self.yaw - heading_to_goal) < self.heading_tol:
                break

        #finally, stop the robot when the distance is moved
        self.velocity_message.angular.z =0
        self.velocity_publisher.publish(self.velocity_message)
    

    def move(self):
            print("Heading towards goal")
            self.velocity_message.linear.x = self.linear_speed 
            loop_rate = rospy.Rate(self.freq)    
            
            while True :
                self.velocity_publisher.publish(self.velocity_message)
                loop_rate.sleep()
                is_reached = abs(self.x - self.x_goal) < self.dist_to_goal_tol_x and abs(self.y - self.y_goal) < self.dist_to_goal_tol_y
                if  self.dist_to_wall < self.dist_to_wall_tol or is_reached or min(self.ranges) < self.dist_to_wall_tol / 1.2: 
                    break
            self.velocity_message.linear.x =0
            self.velocity_publisher.publish(self.velocity_message)
            return is_reached

    def parallel_to_wall (self):
        print("Alligning parallel with the wall")        
        min_ind = self.ranges.index(min(self.ranges))
        self.velocity_message.angular.z = self.angular_speed / 2
        d_ind = min_ind - int(self.range_len /2)
        if  d_ind  > 0:
            dtheta = self.range_increment * d_ind + math.pi / 2
        else:
            dtheta = math.pi / 2 + self.range_increment * d_ind 
        check_ind = 181 
        loop_rate = rospy.Rate(self.freq)

        t0 = rospy.Time.now().to_sec()
        while True :        
            self.velocity_publisher.publish(self.velocity_message)
            t1 = rospy.Time.now().to_sec()
            rotated_angle = (t1-t0) * self.velocity_message.angular.z 
            loop_rate.sleep()
            if  rotated_angle >= dtheta:
                break

        #finally, stop the robot when the distance is moved
        self.velocity_message.angular.z =0
        self.velocity_publisher.publish(self.velocity_message)

    def wall_following(self):        
            print("Following the wall")
            min_side_dist = self.ranges[180]#min(ranges)
            self.velocity_message.linear.x = self.linear_speed 
            loop_rate = rospy.Rate(self.freq) # 
            
            while self.dist_to_wall > self.dist_to_wall_tol / 1.2:
                self.velocity_publisher.publish(self.velocity_message)
                loop_rate.sleep()
                if  abs(min_side_dist - self.ranges[180]) > 1 or self.dist_to_wall_tol * 3 < self.ranges[180]:
                    dist_rot= 0
                    self.velocity_message.angular.z = -math.pi / 5
                    t10 = rospy.Time.now().to_sec()
                    while abs(dist_rot) < math.pi / 3.6 and self.dist_to_wall > self.dist_to_wall_tol / 1.2:
                        self.velocity_publisher.publish(self.velocity_message)
                        loop_rate.sleep()
                        t11 = rospy.Time.now().to_sec()
                        dist_rot = (t11-t10) * self.velocity_message.angular.z

                    dist_trav = 0
                    t00 = rospy.Time.now().to_sec()
                    while dist_trav < 2 and self.dist_to_wall > self.dist_to_wall_tol / 1.2:
                        self.velocity_publisher.publish(self.velocity_message)
                        loop_rate.sleep()
                        t01 = rospy.Time.now().to_sec()
                        dist_trav = (t01-t00) * self.velocity_message.linear.x
                    break
            self.velocity_message.angular.z = 0
            self.velocity_message.linear.x = 0
            self.velocity_publisher.publish(self.velocity_message)
            dist_to_goal = ((self.x - self.x_goal) ** 2 + (self.y- self.y_goal) ** 2) ** 0.5
            angle_diff = self.yaw - math.atan2(self.y_goal-self.y, self.x_goal-self.x)
            diff_ind = int(self.range_len/2) - int(angle_diff / self.range_increment)
            if abs(angle_diff) < 3 * math.pi / 4 and self.ranges[diff_ind] > self.dist_to_wall_tol * 3 :
                    isClear = True
            else:
                isClear = False
            return isClear

    def final_orientation(self):
        print("Rotationg to get to the desired final orientation")
        self.velocity_message.angular.z = self.angular_speed
        loop_rate = rospy.Rate(self.freq)

        while True :
            self.velocity_publisher.publish(self.velocity_message)
            loop_rate.sleep()
            if  abs(self.yaw - self.yaw_goal) < self.heading_tol:
                break

        self.velocity_message.angular.z =0
        self.velocity_publisher.publish(self.velocity_message)

        print("Final Pose: (",self.x, ",", self.y,",",self.yaw*180/math.pi, " deg)") 
        print("Desired Pose: (",self.x_goal, ",", self.y_goal,",",self.yaw_goal*180/math.pi, " deg)") 

if __name__ == '__main__':
    x_goal = 15.45
    y_goal = 6.53
    yaw_goal = 0

    linear_speed = 1.5
    angular_speed_degree = 25
    
    heading_degree_tol = 2
    dist_to_wall_tol = 1
    dist_to_goal_tol_x = 0.5
    dist_to_goal_tol_y = 0.5

    is_reached = False
    isClear = True
    try:
        path_planner = bug0(x_goal, y_goal, yaw_goal, linear_speed, angular_speed_degree, heading_degree_tol, dist_to_wall_tol, dist_to_goal_tol_x, dist_to_goal_tol_y)
        
        while True:
            if isClear:
                path_planner.rotate()
                is_reached = path_planner.move()
            if is_reached:
                path_planner.final_orientation()
                break
            path_planner.parallel_to_wall()
            isClear = path_planner.wall_following()
    except rospy.ROSInterruptException:
        rospy.loginfo("Excepion error - node terminated.")
