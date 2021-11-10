#https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/

import numpy as np
import cv2
  
  
# Capturing video through webcam
webcam = cv2.VideoCapture(0)

_, imageFrame = webcam.read()
dimensions = imageFrame.shape
#screenheight = imageFrame.shape[0] #height doesn't matter - delete if no longer needed
screenwidth = imageFrame.shape[1]
#channels = imageFrame.shape[2] #can delete don't think channels are important - Tim 

#pas width/height

# Start a while loop
def determineBalloonColor():
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
            if(area > 10000): #update for size of balloon
                x, y, w, h = cv2.boundingRect(contour)
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
            if(area > 10000): #update for size of balloon
                x, y, w, h = cv2.boundingRect(contour)
                currentGreenx1 = x
                currentGreeny1 = y
                currentGreenx2 = x+w
                currentGreeny2 = y+h
                result = isCentered(currentGreenx1,currentGreenx2,screenwidth)
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
    centerX = width/2
    centerBoxX = (x2-x1)/2

    centerXLowRange = centerX - 40
    centerXHighRange = centerX + 40  
    if(centerBoxX+width > centerXLowRange and centerBoxX+width > centerXHighRange):
        return 1 #centered
    else:
        if(centerBoxX+width < centerXLowRange):
            return 2 #object too far left, turn towards the left
        else:
            return 3 #object too far right, turn towards the right

def main():
    determineBalloonColor()

if __name__ == "__main__":
    main()
