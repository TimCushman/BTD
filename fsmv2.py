#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Authors: Tim Cushman and Anna Manning, Ben Lehrburger and Jack Gnibus 
# Date: November 2021

# Import of python modules.
import math # use of pi.
import random # to find random angle 
import numpy as np
import cv2
from cv_bridge import CvBridge 


# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan, CompressedImage # message type for scan
from enum import Enum

# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan'
DEFAULT_IMAGE_TOPIC = "/camera/rgb/image_raw/compressed"
#DEFAULT_IMAGE_TOPIC = "/camera/rgb/image_rect_color"

# Frequency at which the loop operates
FREQUENCY = 3 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.1 # m/s
ANGULAR_VELOCITY = math.pi/8 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
MIN_THRESHOLD_DISTANCE = .25 # m, threshold distance, should be smaller than range_max
MIN_THRESHOLD_DISTANCE_IN_POP_STATE = .05 # Field of view in radians that is checked in front of the robot (feel free to tune)
MIN_SCAN_ANGLE_RAD = -30.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = +30.0 / 180 * math.pi

# Sets up our fsm with our different states
class fsm(Enum):
    RANDOM_WALK = 1
    GREEN_BALLOON = 2
    TURN = 3
    STOP = 4

# Implement fsm 
class BalloonPopper():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        min_threshold_distance_in_pop_state = MIN_THRESHOLD_DISTANCE_IN_POP_STATE, scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD]):
        """Constructor."""
        
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        # image subscriber 
        self._img_sub = rospy.Subscriber(DEFAULT_IMAGE_TOPIC, CompressedImage, self.image_callback, queue_size=1, buff_size=52428800)
        self.image = None
    
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.min_threshold_distance_in_pop_state = min_threshold_distance_in_pop_state 

        # start off fsm in random walk state 
        self._fsm = fsm.RANDOM_WALK

        # Flag used to control the behavior of the robot.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.

        # flags for state 
        self.red = False
        self.green = False
        self.popped = False

        # to check if the green baloon is in the center of the frame
        self.center = False

    def _laser_callback(self, msg):
        # Only look at the range values within the field of view and find the minimum 
        min_index_left = msg.ranges[0:60]
        min_index_right = msg.ranges[-60:]
        range_values = min_index_left + min_index_right

        # If the minimum range value is closer to min_threshold_distance, change the flag self._close_obstacle
        min_range = min(range_values)

        # If it doesnt see a green balloon use a larger distance from the wall.
        if not self.green:
            if min_range < self.min_threshold_distance:
                print("Too close to wall, will turn")
                self.stop()
                self._close_obstacle = True 

        else: # If in green balloon state allow it to get closer so it can pop the balloon 
            if min_range < self.min_threshold_distance_in_pop_state:
                self.stop()
                print("Too close to balloon/wall, will turn")
                self._fsm = fsm.TURN
                      
    def image_callback(self, img_msg):
            rospy.loginfo(img_msg.header)
            img_arr = np.fromstring(img_msg.data, np.uint8)
            image_res = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
            self.image = cv2.cvtColor(image_res, cv2.COLOR_RGB2BGR)
            
            # Call the code that determines the balloon color after each image is read in
            self.determineBalloonColor()


    # The move function is from PA 0
    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)
    # The stop function is from PA 0
    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)
    
    # The rotate relative function is adapted from PA 4 code
    def rotate_rel(self,angle, type = None):
        # If the robot sees a green balloon while in the rotate state, stop and track the balloon.  
        if self._fsm == fsm.GREEN_BALLOON:
            self.stop()

        rate = rospy.Rate(FREQUENCY)

        start_time = rospy.get_rostime()
        
        while not rospy.is_shutdown():
            if self._fsm == fsm.GREEN_BALLOON and type == None:
                self.stop()
                break
            duration = abs(angle/self.angular_velocity)
            if rospy.get_rostime() - start_time <= rospy.Duration(duration):
                if angle < 0: 
                    self.move(0, - self.angular_velocity)
                else: 
                    self.move(0, self.angular_velocity)
            else:
                break 
            
            # Sleep to keep the set frequency.
            rate.sleep()

    # The random walk code was adapted from the PA 0 code to deal with getting and turning a random angle
    # between pi/3 and pi.

    def random_walk(self):
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
          
            # If the flag is set to false, we are not near a wall and will walk forwards at the speed of the linear velocity 
            if (self._close_obstacle == False): 
                self.move(self.linear_velocity, 0)
            else:
                randomAngle = random.uniform(math.pi/3,math.pi) # Get a random angle 
                timeToRun = abs(randomAngle)/ANGULAR_VELOCITY # This line gives us the time needed to turn the correct angle amount
                start_time = rospy.get_rostime() # Start a timer
                while True:                      
                    if(randomAngle < 0): # If the angle is less than zero, turn clockwise
                        self.move(0, self.angular_velocity)
                    else:
                        self.move(0, -self.angular_velocity) # If the random angle is greater than zero, turn counter clockwise
                    self._close_obstacle = False # After moving, set the flag to false because we can now move forwards without running into a wall
                    
                    # Once the runtime is greater than or equal to the time needed to turn the correct angle amount, we break out of the while loop 
                    if rospy.get_rostime() - start_time >= rospy.Duration(timeToRun): 
                        break      
           
            rate.sleep()
 

    # The stop function is adapted from PA 0. 

    def translate(self, d, direction):
        # Rate at which to operate the while loop.
        rate = rospy.Rate(FREQUENCY)
        start_time = rospy.get_rostime()
        linearVel = self.linear_velocity

        while not rospy.is_shutdown():
            # Check if done.
            if not self._close_obstacle and linearVel < 0:
                duration = rospy.Duration(abs(d/self.linear_velocity))

                if rospy.get_rostime() - start_time <= duration:
                    self.move(linearVel, 0)
                else:
                    break 
            else: 
                break 
            
            # Sleep to keep the set frequency.
            rate.sleep()

    # Deals with moving between FSM states and carrying out the proper code within each state. 
    def pop(self):
        rate = rospy.Rate(10) # loop at 10 Hz.
        while not rospy.is_shutdown():

                if self._fsm == fsm.RANDOM_WALK:
                    self.random_walk()

                # Go backwards and turn 60 degrees in the TURN state
                if self._fsm == fsm.TURN:
                    self.stop()
                    self.rotate_rel(math.pi/3)
                    
                    # Set flags back to False after turning and set FSM state to Random Walk
                    self.red = False
                    self.green = False
                    self._fsm = fsm.RANDOM_WALK

                if self._fsm == fsm.GREEN_BALLOON:
                    # If in Green Balloon state move forwards, lidar will stop the robot if it is too close to a wall or a balloon
                    if not self._close_obstacle:   
                        self.move(self.linear_velocity, 0)
                    else: 
                        self.stop()
                        self._fsm = fsm.TURN
                
                rate.sleep()


# The follow code within the function determineBalloonColor() was created by GeeksForGeeks user
# @goodday451999 Last Updated : 10 May, 2020. The code has been since adapted get rid of certain unimportant functionality
# and only sense objects of a ceratin size (so we don't mark all areas of green or red as balloons. A link to the original posting is:  
# https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/


    def determineBalloonColor(self):
        imageFrame = self.image
        dimensions = imageFrame.shape

        screenwidth = imageFrame.shape[1]
        
        # Convert the imageFrame in BGR(RGB color space) to HSV(hue-saturation-value) color space
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_RGB2HSV)

        # Set range for red color and define mask
        red_lower = np.array([136, 87, 111], np.uint8)
        red_upper = np.array([180, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)    

        # Set range for green color and define mask
        green_lower = np.array([25, 52, 72], np.uint8)
        green_upper = np.array([102, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

        # Morphological Transform, Dilation for each color and bitwise_and operator between imageFrame and mask determines to detect only that particular color
        kernal = np.ones((5, 5), "uint8")
        
        # For red color
        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = red_mask)
        # For green color
        green_mask = cv2.dilate(green_mask, kernal)
        res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                    mask = green_mask)

        # Creating contour to track green color
        _,contours, hierarchy = cv2.findContours(green_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        
        # We set green and red flags to false before we loop through all th contours within th frame. 
        self.green = False
        self.red = False
        
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 4000): # Updated area for our balloons so we don't mark all small areas as green
                # Get x value and width of box so that we can tell if the robot is centered
                x, y, w, h = cv2.boundingRect(contour)
                currentGreenx1 = x
                currentGreenx2 = x+w
                self.green = True
                print("Green Balloon Detected")
                
                # Call functiont to check if the balloon is centered
                self.isCentered(currentGreenx1,currentGreenx2,screenwidth)
        # If after looping through all contours no green area of a large enough size was seen stay or go back to random walk. 
        # (After a balloon is popped it will now not see green and go back to random walk)
        if not self.green: 
            self._fsm = fsm.RANDOM_WALK

                
        # Creating contour to track red color
        _, contours, hierarchy = cv2.findContours(red_mask,
                                                cv2.RETR_TREE, 
                                                cv2.CHAIN_APPROX_SIMPLE)
        
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 4000): # Updated area for our balloons so we don't mark all small areas as red
                x, y, w, h = cv2.boundingRect(contour)
                if(self.green != True):
                    self.red = True 
                    print("Red Balloon Detected")
                    self.stop()
                    self.rotate_rel(math.pi/2)
                    self.red = False
            
   
   # This function checks if is centered within the image frame which is centered on the robot. If it is not centered
   # it will calculate and turn the angle and direction needed to rotate to center on the balloon.
    def isCentered(self,x1,x2,width):
        buffer = 80 # Set to help give a range to be within the center
        
        # Get center of the screen 
        centerX = width/2

        # Calculate where the center of the box around the balloon is within the image frame
        centerBoxX = x1 + ((x2-x1)/2)
        
        # Calculate a lower and upper range that we consider "centered". This allows it to not be perfect and still move
        # forwards with popping the balloon. 
        centerXLowRange = centerX - buffer
        centerXHighRange = centerX + buffer        

        # If the bouding box around the balloon is within our centered range, mark it as centered and move forwards.
        if(centerBoxX < centerXHighRange and centerBoxX > centerXLowRange):
            print("Centered")
            self.center = True
            self._fsm = fsm.GREEN_BALLOON
        
        else:
            # Calculate what angle and direction the robot has to turn to get the balloon in the center of the image
            rotation_angle = abs(float(centerX-centerBoxX))/float(width)*(math.pi/3) 
            
            if(centerBoxX <= centerXLowRange):
                print("balloon too Left") 
                self.center = False
                self.rotate_rel(rotation_angle, "balloon")
                self._fsm = fsm.GREEN_BALLOON
            else:
                print("balloon too Right")
                self.center = False
                self.rotate_rel(-rotation_angle, "balloon")
                self._fsm = fsm.GREEN_BALLOON

    # This helps the program continue runnning even if the pop loop breaks. 
    def go(self):
        self.pop()
        rospy.spin() 

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
        
        balloon_popper.go()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
