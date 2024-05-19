#!/usr/bin/env python
import cv2 as cv
import numpy as np
import time

import rospy
from std_msgs.msg import Int16MultiArray

MAX_OFFSET_RATE = 90
MAX_AREA_RATE = 2700

# ======================================
# === Global Functions and Constants ===
# ======================================

def map_val(input, input_min, input_max, output_min, output_max):
    input_span = input_max - input_min
    output_span = output_max - output_min

    # Convert the left range into a 0-1 range (float)
    valueScaled = (input - input_min) / float(input_span)

    # Convert the 0-1 range into a value in the right range.
    output = output_min + (valueScaled * output_span)
    return output


# ======================================
# === Main function ====================
# ======================================

def main():
    rospy.init_node("rc_controller", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Int16MultiArray, queue_size=10)
    rate = rospy.Rate(60) # 60 Hz (ROS message) / FPS (Images)
    
    linear_vel = 0
    angular_vel = 0
    previous_offset = None
    previous_area = None

    # Start video capture 
    source = cv.VideoCapture(0) 
    source.set(3,1280)
    source.set(4,720)

    message = Int16MultiArray() 
    

    while not rospy.is_shutdown():
        # Read a frame
        isTrue, frame = source.read() 

        if not isTrue:
            break 

        # Convert the frame to HSV color space
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # define color range
        lower_color = np.array([10,192,107])
        upper_color = np.array([179,255,255])

        # Create a mask 
        mask = cv.inRange(hsv, lower_color, upper_color)

        # Find contours of the masked area
        contours, _ = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) 
        # print(contours)
        if len(contours) > 0:

            # Find the largest contour (the ball)
            ball_contour = max(contours, key=cv.contourArea)

            # Estimate area
            area = cv.contourArea(ball_contour)
            

            # Get center & radius
            ((x, y), radius) = cv.minEnclosingCircle(ball_contour)
    
            # shift to circle center to the center of img
            offset = int(x - (frame.shape[1]/2))
            
                

            # calculate offset rate
            if previous_offset is not None:
                offset_rate = offset - previous_offset
                angular_vel = map_val(offset_rate, -MAX_OFFSET_RATE, MAX_OFFSET_RATE, -1, 1)

                if angular_vel > 1:
                    angular_vel = 1
                elif angular_vel < -1:
                    angular_vel = -1

            # calculate area rate
            if previous_area is not None:
                area_rate = area - previous_area
                linear_vel = map_val(area_rate, -MAX_OFFSET_RATE, MAX_OFFSET_RATE, -1, 1)
                
                if linear_vel > 1:
                    linear_vel = 1 

                elif linear_vel < -1:
                    linear_vel = -1
            
            print("linear: {:.3f}\tangular: {:.3f}".format(linear_vel, angular_vel))
            
            ## proportional control (mapping)
            if offset < 0: 
                # left
                direction = -1
                velocity = int(map_val(offset, -320, 0, 150, 0))
                message.data = [velocity, direction]
                # pub.publish(message)

            elif offset > 0:
                # right
                direction = 1
                velocity = int(map_val(offset, 0, 320, 0, 150))
               
                message.data = [velocity, direction]
                # pub.publish(message)

            else:
                #stop
                message.data = [0, 0]
                # pub.publish(message)

            # Draw a circle around the ball
            if radius > 10:
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

            previous_offset = offset
            previous_area = area

        # Display the resulting frame
        cv.imshow("Ball Tracking", frame)
        # cv.imshow('mask',mask)
        # Exit the loop if 'q' is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        
        

        message.data = [100, -1]
        pub.publish(message)

        rate.sleep()

        


    # Release the video capture and close all windows
    source.release()
    cv.destroyAllWindows()

if __name__ == "__main__":

    main()

