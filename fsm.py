#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Anna Manning 
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
from sensor_msgs.msg import LaserScan # message type for scan
from enum import Enum

from sensor_msgs.msg import Image, CompressedImage


# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan'
DEFAULT_IMAGE_TOPIC = "/camera/rgb/image_raw/compressed"
#DEFAULT_IMAGE_TOPIC = "/camera/rgb/image_rect_color"

# Frequency at which the loop operates
FREQUENCY = 3 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.1 # m/s
ANGULAR_VELOCITY = math.pi/16 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
MIN_THRESHOLD_DISTANCE = .25 # m, threshold distance, should be smaller than range_max

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
    STOP = 4


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


        # image subscriber 
        self._img_sub = rospy.Subscriber(DEFAULT_IMAGE_TOPIC, CompressedImage, self.image_callback)
        #self._img_sub = rospy.Subscriber(DEFAULT_IMAGE_TOPIC, Image, self.image_callback, queue_size=1)
    
        self.image = None

        self.bridge = CvBridge()

        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle

        self._fsm = fsm.RANDOM_WALK

        # Flag used to control the behavior of the robot.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.
        self._pop_close_obstacle = False

        self.red = False
        self.green = False
        self.popped = False

        # to check if the green baloon is in the center of the frame
        self.center = False
        self.left = False
        self.right = False
        self.min_range = 100
        self.pop_dist = .25
        self.pop_thresh_dist = .15

    def _laser_callback(self, msg):
        # TODO: laser callback function (some of this might have to go in camera callback?)
        # deal with fsm states 

        # robot will only be random walking when green flag and red flag are false, thus only needs to run this loop when 
        # in the process of random walking 
        min_index = int((self.scan_angle[0] - msg.angle_min) / msg.angle_increment)
        max_index = int((self.scan_angle[1] - msg.angle_min) / msg.angle_increment)

        min_index_left = msg.ranges[0:30]
        min_index_right = msg.ranges[-30:]
        range_values = min_index_left + min_index_right

        # only look at the range values within the scan angle and find the minimum 
       # range_values = msg.ranges[min_index:max_index + 1]

        # If the minimum range value is closer to min_threshold_distance, change the flag self._close_obstacle
        self.min_range = min(range_values)

        # if not self._close_obstacle and self._fsm == fsm.RANDOM_WALK:
        #     if min_range < self.min_threshold_distance:
        #         self._close_obstacle = True 

        #if self._fsm == fsm.GREEN_BALLOON:
        if self.min_range < self.min_threshold_distance: 
            print("too close to wall")
            self._close_obstacle = True

        if self.min_range < self.pop_thresh_dist:
            print("too close too balloon")
            self._pop_close_obstacle = True 

                   
            
    def image_callback(self, img_msg):
        if self._fsm != fsm.TURN :
            rospy.loginfo(img_msg.header)
            #self.image = self.br.imgmsg_to_cv2(img_msg,desired_encoding='8UC3')
            
            img_arr = np.fromstring(img_msg.data, np.uint8)
            img = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
            self.image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
            #self.image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            
            self.determineBalloonColor()


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
    
    def rotate_rel(self,angle, method):
        # Rate at which to operate the while loop.
        # if self._fsm != fsm.RANDOM_WALK:
        #     print("HERE")
        #     self.stop()
        rate = rospy.Rate(FREQUENCY)
    

        start_time = rospy.get_rostime()
        
        while not rospy.is_shutdown():


            # Check if done.
            duration = abs(angle/self.angular_velocity)
            if rospy.get_rostime() - start_time <= rospy.Duration(duration):
                # exit out of turning 

                if self.green and method == "random_walk":
                    self.stop()
                    return 
                if angle < 0: 
                    self.move(0, - self.angular_velocity)
                else: 
                    self.move(0, self.angular_velocity)
            else:
                break 
            
            # Sleep to keep the set frequency.
            rate.sleep()
    
        #rospy.sleep(1)

    # code adapted from pa0
    def random_walk(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            # break out of loop if a green balloon is sensed
            self.rotate_rel(60.0 / 180.0 * math.pi, "random_walk")

            if self.green or self.red:
                return  

            if not self._close_obstacle:  
                dist = random.random()
                self.translate(dist)

            if self.green or self.red:
                return  
            else: 
                # stop the robots motion
                self.stop()
                self.rotate_rel(120.0 / 180.0 * math.pi,"random_walk")

                self._close_obstacle = False


            if self.green or self.red:
                return  
                    

            rate.sleep()

        rospy.sleep(1)

    def translate(self, d):
        # Rate at which to operate the while loop.
        rate = rospy.Rate(FREQUENCY)
        start_time = rospy.get_rostime()
        while not rospy.is_shutdown():
            # Check if done.
            
            duration = rospy.Duration(abs(d/self.linear_velocity))

            
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
                self.random_walk()

            if self._fsm == fsm.TURN:
                self.rotate_rel(math.pi/3, "turn")

                # set state back to random walk after turning 
                self.red = False
                self.green = False
                rospy.sleep(1)
                self._fsm = fsm.RANDOM_WALK


            if self._fsm == fsm.GREEN_BALLOON:
                print("Pop Green")
                # go forward, lidar should stop the robot 
                if not self._pop_close_obstacle: 
                    self.move(self.linear_velocity, 0)
                else: 
                    self.green = False
                    print("changing state")
                    self._fsm = fsm.TURN
                    


            rate.sleep()
   
   
    def go(self):
        self.pop()
        rospy.spin()



    # Start a while loop
    def determineBalloonColor(self):
        # Capturing video through webcam
        #webcam = cv2.VideoCapture(0)
        imageFrame = self.image
        #_, imageFrame = webcam.read()
        dimensions = imageFrame.shape
        
        #screenheight = imageFrame.shape[0] #height doesn't matter - delete if no longer needed
        screenwidth = imageFrame.shape[1]
        
        # Convert the imageFrame in 
        # BGR(RGB color space) to 
        # HSV(hue-saturation-value)
        # color space
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    
        # Set range for red color and 
        # define mask

        # red_lower = np.array([136, 87, 111], np.uint8) #RED
        # red_upper = np.array([180, 255, 255], np.uint8)  #RED 

        red_lower = np.array([94, 80, 2], np.uint8) #marks as red but checks for blue 
        red_upper = np.array([120, 255, 255], np.uint8) #marks as red but checks for blue
        
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
        c = cv2.findContours(red_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        # Creating contour to track red color
        _, contours, hierarchy = cv2.findContours(red_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        # for pic, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     if(area > 4000): #updated for size of balloon
        #         x, y, w, h = cv2.boundingRect(contour)
        #         currentRedx1 = x
        #         currentRedx2 = x+w
 
        #         #print("SEES RED in area function")  
        #         self.red = True 
        #         print("BLUE")
        #         self._fsm = fsm.TURN
                
    
        # Creating contour to track green color
        _,contours, hierarchy = cv2.findContours(green_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 4000): #updated for size of balloon
                x, y, w, h = cv2.boundingRect(contour)
                currentGreenx1 = x
                currentGreenx2 = x+w
                #print("SEES GREEN IN AREA Function")
                self.green = True
                print("GREEN")
                rospy.sleep(1)
                self.isCentered(currentGreenx1,currentGreenx2,screenwidth)
                
            else: 
                self.green = False




    def isCentered(self,x1,x2,width):
        buffer = 20 #set to help give a range to be within the center
        centerX = width/2
        centerBoxX = x1 + ((x2-x1)/2)

        centerXLowRange = centerX - buffer
        centerXHighRange = centerX + buffer

        rotation_angle = abs(float(centerX-centerBoxX))/float(width)*(math.pi/3)
        print(centerBoxX,"center box")
        
        # print(rotation_angle, "angle of rotation to get to center")

        if(centerBoxX <= centerXHighRange and centerBoxX >= centerXLowRange) or self.min_range < .5:
            print("Centered") #uncomment for easier testing of centering
            #return 1 #centered
            self.center = True
            self._fsm = fsm.GREEN_BALLOON
        else:
            
            if(centerBoxX < centerXLowRange):
                print("tooLeft") #uncomment for easier testing of centering
                #return 2 #object too far left, turn towards the left
                self.center = False
                self.rotate_rel(rotation_angle, "green")
                #rospy.sleep(1)
                self._fsm = fsm.GREEN_BALLOON


            else:
                print("tooRight") #uncomment for easier testing of centering
                #return 3 #object too far right, turn towards the right
                self.center = False
                self.rotate_rel(-rotation_angle, "green")
                #rospy.sleep(1)
                self._fsm = fsm.GREEN_BALLOON




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
