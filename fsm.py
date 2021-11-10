#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Anna Manning 
# Date: November 2021

# Import of python modules.
import math # use of pi.
import random # to find random angle 


# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from enum import Enum
from random_walk import RandomWalk

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
MIN_THRESHOLD_DISTANCE = 1.0 # m, threshold distance, should be smaller than range_max

# Field of view in radians that is checked in front of the robot (feel free to tune)
MIN_SCAN_ANGLE_RAD = -30.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = +30.0 / 180 * math.pi




class fsm(Enum):
    RANDOM_WALK = 1
    GREEN_BALLOON = 2
    TURN = 3
    RETRY = 4
    TURN_RIGHT = 5
    TURN_LEFT = 6

# implement fsm 
class BalloonPopper():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD]):
        """Constructor."""

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle


        self.curr_linear_vel = self.linear_vel
        self._fsm = fsm.RANDOM_WALK

        # Flag used to control the behavior of the robot.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.

        self.red = False
        self.green = False
        self.popped = False

        # to check if the green baloon is in the center of the frame
        self.center = False
        self.left = False
        self.right = False

    def _laser_callback(self, msg):
        # TODO: laser callback function (some of this might have to go in camera callback?)
        # deal with fsm states 

        if not self._close_obstacle:
            # angle increment of the scanner 
            angle_increment = msg.angle_increment
            # find the number of entries in the ranges array from the scan  message 
            total_size= int((msg.angle_max - msg.angle_min)//angle_increment)

            # find the number of indices below 0 and above 0 to check in the ranges 
            scan_num_indices_more0 = int((MAX_SCAN_ANGLE_RAD//angle_increment))
            scan_num_indices_less0 = int(MIN_SCAN_ANGLE_RAD//angle_increment)

            # the index of angle zero is in the middle of the ranges array, so to find the proper
            # min and max index find the middle index of the total ranges array and use the values
            # calucalted above to find the index
            min_index = total_size//2 + scan_num_indices_less0
            max_index = total_size//2 + scan_num_indices_more0

            # only look at the range values within the scan angle and find the minimum 
            range_values = msg.ranges[min_index:max_index + 1]

            # If the minimum range value is closer to min_threshold_distance, change the flag self._close_obstacle
            min_range = min(range_values)

            if min_range < self.min_threshold_distance:
                self._close_obstacle = True 
            
            ####### ANSWER CODE END #######

# from pa0
    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)
    
    def rotate_rel(self,angle):
        # Rate at which to operate the while loop.
        rate = rospy.Rate(FREQUENCY)

        start_time = rospy.get_rostime()
        
        while not rospy.is_shutdown():
            #print("turning")
            # Check if done.
            duration = abs(angle/self.angular_vel)
            if rospy.get_rostime() - start_time <= rospy.Duration(duration):
                if angle < 0: 
                    self.move(0, - self.angular_vel)
                else: 
                    self.move(0, self.angular_vel)
            else:
                break 
            
            # Sleep to keep the set frequency.
            rate.sleep()
    
        rospy.sleep(1)
    
    def random_walk(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C
            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random amount of time
            # after which the flag is set again to False.
            # Use the function move already implemented, passing the default velocities saved in the corresponding class members.
            ####### TODO: ANSWER CODE BEGIN #######
            if not self._close_obstacle:  
                self.move(LINEAR_VELOCITY, 0)

            else: 
                # stop the robots motion 
                self.stop()

                # find a random angle between (-pi,pi)
                new_angle = random.uniform(- math.pi, math.pi)

                # find the absolute value of the time to see how long the robot should rotate for 
                # to reach the desired angle 
                time = abs(new_angle/self.angular_velocity)
                
                # adapted from lec03_example_go_forward.py 
                start_time = rospy.get_rostime()

                while True: 
                    if rospy.get_rostime() - start_time >= rospy.Duration(time):
                        break          
                    # if the random angle is negative, rotate counterclockwise 
                    if new_angle <= 0: 
                        self.move(0, self.angular_velocity)
                    # if the angle is positive rotate clockwise 
                    else: 
                        self.move(0, - self.angular_velocity)

                self.stop()
                # reset _close_obstacle 
                self._close_obstacle = False              
            ####### ANSWER CODE END #######
            rate.sleep()

    def pop(self):
        # TODO
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            if self._fsm == fsm.RANDOM_WALK:
                # call random walk
                if self.green: 
                    self._fsm = fsm.GREEN_BALLOON

                if self.red:
                    self._fsm = fsm.TURN
                else:
                    # do random walk 

            if self._fsm == fsm.TURN:
                self.rotate_rel(math.pi)
                # set state back to random walk after turning 
                self._fsm == fsm.RandomWalk


            if self._fsm == fsm.GREEN_BALLOON:
                # accelerate the robot, remember to reset the velovity after popping
                if self.centered:
                    self.curr_linear_vel = self.linear_velocity*2
                    self.move(self.curr_linear_vel,0)
                if self.left:
                    self.move(0,self.angular_vel)
                if self.right:
                    self.move(0, -self.angular_vel)




def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("random_walk")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the balloon popper 
    balloon_popper = BalloonPopper()
    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(balloon_popper.stop)

    # Robot finds green balloons, and pops them, avoiding red ballons
    try:
        balloon_popper.pop()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
