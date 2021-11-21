#Author: Timothy, Anna, Jack and Ben
#Date: 11/11/21
#Class: CS81 - Final Project
#Program: Color Sensing with check if object is centered in the screen
#Inputs: A live video stream
#Outputs: If there is a red or green object present within the current frame. If the object is green, 
#there is also a check if that object is centered within the sreen. 

# The follow code within the function determineBalloonColor() was created by GeeksForGeeks user
# @goodday451999 Last Updated : 10 May, 2020. The code has been since adapted to selecte a better range 
# of values for our balloon colors and to make the code work better for our project. A link to the original
# posting is:  
# https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/


import numpy as np
import cv2
  

# Start a while loop
def determineBalloonColor():
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
                result = isCentered(currentGreenx1,currentGreenx2,screenwidth)
                
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

def isCentered(x1,x2,width):
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
        return 1 #centered
    else:
        if(centerBoxX < centerXLowRange):
            #print("tooLeft") #uncomment for easier testing of centering
            return 2 #object too far left, turn towards the left
        else:
            #print("tooRight") #uncomment for easier testing of centering
            return 3 #object too far right, turn towards the right

def main():
    determineBalloonColor()

if __name__ == "__main__":
    main()
