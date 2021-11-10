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

#from lec_08_pa0_random_walk_fsm.py
class fsm(Enum):
    RANDOM_WALK = 1
    #RED_BALLON = 2
    GREEN_BALLOON = 2
    TURN = 3
    RETRY = 4

# implement fsm 
class BalloonPopper():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY):
        """Constructor."""

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        self.angular_vel = angular_velocity

        self.linear_vel = linear_velocity
        self.curr_linear_vel = self.linear_vel

        self._fsm = fsm.RANDOM_WALK

        self.red = False
        self.green = False
        self.popped = False

    def _laser_callback(self, msg):
        # TODO: laser callback function (some of this might have to go in camera callback?)
        # deal with fsm states 

        if self._fsm == fsm.GREEN_BALLOON:
            # sense when robot gets too close to the balloon without popping, then retry also sense 
            # if robot successfully pops balloon 
            # will have to work with camera here, somehow set popped
            if self.popped:
                self._fsm == fsm.TURN

        if self._fsm == fsm.RandomWalk:
            # use sensors to look for red/ green balloons 
            if self.red:
                self._fsm == fsm.TURN

            if self.green:
                self._fsm == fsm.GREEN_BALLOON





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
    
    def pop(self):
        # TODO
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            if self._fsm == fsm.RANDOM_WALK:
                # call random walk
                random_walk = RandomWalk(linear_velocity=self.linear_vel, angular_velocity=self.angular_vel, min_threshold_distance=MIN_THRESHOLD_DISTANCE, scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD])
                random_walk.spin()

            if self._fsm == fsm.TURN:
                self.rotate_rel(math.pi)
                # set state back to random walk after turning 
                self._fsm == fsm.RandomWalk


            if self._fsm == fsm.GREEN_BALLOON:
                # accelerate the robot, remember to reset the velovity after popping
                self.curr_linear_vel = self.linear_velocity*2
                self.move(self.curr_linear_vel,0)




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
