#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Anna Manning 
# Date: November 2021

# Import of python modules.
import math # use of pi.
import random # to find random angle 
import numpy as np
import cv2


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
LINEAR_VELOCITY = 0.3 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
MIN_THRESHOLD_DISTANCE = 1.0 # m, threshold distance, should be smaller than range_max

# Field of view in radians that is checked in front of the robot (feel free to tune)
MIN_SCAN_ANGLE_RAD = -30.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = +30.0 / 180 * math.pi



# maybe add a state "popped" that the robot will go into right after it pops the balloon 
# so it doesnt run into a wall with the knife, might not need retry bc knife is sharp, 
# could also just enter into turn state right after it popps, so it turns robot 180 and continues to random walk 
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

        # robot will only be random walking when green flag and red flag are false, thus only needs to run this loop when 
        # in the process of random walking 
        if not self._close_obstacle and not self.green and not self.red:
            min_index = int((self.scan_angle[0] - msg.angle_min) / msg.angle_increment)
            max_index = int((self.scan_angle[1] - msg.angle_min) / msg.angle_increment)

            # only look at the range values within the scan angle and find the minimum 
            range_values = msg.ranges[min_index:max_index + 1]

            # If the minimum range value is closer to min_threshold_distance, change the flag self._close_obstacle
            min_range = min(range_values)
            if min_range < self.min_threshold_distance:
                self._close_obstacle = True 
            

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

    # code adapted from pa0
    def random_walk(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            # break out of loop if a green/red balloon is sensed
            if self.green or self.red: 
                break

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

            rate.sleep()

    def translate(self, d):
        # Rate at which to operate the while loop.
        rate = rospy.Rate(FREQUENCY)
        start_time = rospy.get_rostime()
        while not rospy.is_shutdown():
            # Check if done.
            
            duration = rospy.Duration(abs(d/self.linear_velocity))
            #print(duration)
            
            if rospy.get_rostime() - start_time <= duration:
                self.move(self.linear_velocity, 0)
            else:
                break 
            
            # Sleep to keep the set frequency.
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
                    self.random_walk()

            if self._fsm == fsm.TURN:
                self.rotate_rel(math.pi)
                # set state back to random walk after turning 
                self.red = False
                self._fsm == fsm.RandomWalk
                


            if self._fsm == fsm.GREEN_BALLOON:
                # accelerate the robot, remember to reset the velovity after popping
                if self.centered:
                    # for now just move a little forward and then stop and continue random walk 
                    #self.curr_linear_vel = self.linear_velocity*1.5
                    #self.move(self.curr_linear_vel,0)
                    self.translate(.3)
                    self.green = False 
                    self._fsm = fsm.TURN

                if self.left:
                    self.move(0,self.angular_velocity)
                if self.right:
                    self.move(0, -self.angular_velocity)

            rate.sleep()



    # Start a while loop
    def determineBalloonColor(self):
        # Capturing video through webcam
        webcam = cv2.VideoCapture(0)

        _, imageFrame = webcam.read()
        dimensions = imageFrame.shape
        #screenheight = imageFrame.shape[0] #height doesn't matter - delete if no longer needed
        screenwidth = imageFrame.shape[1]
        #print(screenwidth,"WIDTH") #should be 640 
        #channels = imageFrame.shape[2] #can delete don't think channels are important - Tim 
        while(1):
            # Reading the video from the
            # webcam in image frames
            _, imageFrame = webcam.read()

            # height, width, number of channels in image

            
            # Convert the imageFrame in 
            # BGR(RGB color space) to 
            # HSV(hue-saturation-value)
            # color space
            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        
            # Set range for red color and 
            # define mask

            # red_lower = np.array([5, 50, 50], np.uint8) #USE TO TEST ON ORANGE COLORS - WILL STILL MARK AS RED
            # red_upper = np.array([15, 255, 255], np.uint8) #USE TO TEST ON ORANGE COLORS - WILL STILL MARK AS RED

            red_lower = np.array([136, 87, 111], np.uint8)
            red_upper = np.array([180, 255, 255], np.uint8)
            red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        
            # Set range for green color and 
            # define mask
            green_lower = np.array([25, 52, 72], np.uint8)
            green_upper = np.array([102, 255, 255], np.uint8)
            green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
        

            # Morphological Transform, Dilation
            # for each color and bitwise_and operator
            # between imageFrame and mask determines
            # to detect only that particular color
            kernal = np.ones((5, 5), "uint8")
            
            # For red color
            red_mask = cv2.dilate(red_mask, kernal)
            res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                                    mask = red_mask)
            
            # For green color
            green_mask = cv2.dilate(green_mask, kernal)
            res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                        mask = green_mask)

            # Creating contour to track red color
            contours, hierarchy = cv2.findContours(red_mask,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)
            
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 10000): #updated for size of balloon
                    x, y, w, h = cv2.boundingRect(contour)
                    currentRedx1 = x
                    currentRedx2 = x+w
                    result = isCentered(currentRedx1,currentRedx2,screenwidth)
                    # pass results to another function
                    
                    imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                            (x + w, y + h), 
                                            (0, 0, 255), 2) 
                    
                    cv2.putText(imageFrame, "Red Color", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                                (0, 0, 255))    
        
            # Creating contour to track green color
            contours, hierarchy = cv2.findContours(green_mask,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)
            
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 10000): #updated for size of balloon
                    x, y, w, h = cv2.boundingRect(contour)
                    currentGreenx1 = x
                    currentGreenx2 = x+w
                    result = self.isCentered(currentGreenx1,currentGreenx2,screenwidth)
                    
                    # pass results to another function

                    imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                            (x + w, y + h),
                                            (0, 255, 0), 2)
                    
                    cv2.putText(imageFrame, "Green Color", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                1.0, (0, 255, 0))
        

            # Program Termination
            cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                break

    def isCentered(self,x1,x2,width):
        buffer = 15 #set to help give a range to be within the center
        centerX = width/2
        centerBoxX = x1 + ((x2-x1)/2)

        centerXLowRange = centerX - buffer
        centerXHighRange = centerX + buffer

        # Test print statements to help with centering
        # print(centerBoxX,'BW')
        # print(centerXLowRange,"LOW")
        # print(centerXHighRange,"HIG")

        if(centerBoxX < centerXHighRange and centerBoxX > centerXLowRange):
            #print("Centered") #uncomment for easier testing of centering
            #return 1 #centered
            self.center = True
            self.right = False
            self.left = False
        else:
            if(centerBoxX < centerXLowRange):
                #print("tooLeft") #uncomment for easier testing of centering
                #return 2 #object too far left, turn towards the left
                self.center = False
                self.right = False
                self.left = True
            else:
                #print("tooRight") #uncomment for easier testing of centering
                #return 3 #object too far right, turn towards the right
                self.center = False
                self.right = True
                self.left = False




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
