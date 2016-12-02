#!/usr/bin/env python

"""
Author: James Irwin (james.irwin@wsu.edu)
Description:
    Example code for getting started with the ardrone
"""

import rospy
from sensor_msgs.msg import Image #for recieving video feed
from geometry_msgs.msg import Twist # controlling the movements
from std_msgs.msg import Empty #send empty message for takeoff and landing
import numpy 
import cv2
from image_converter import ToOpenCV

class QuadcopterController:
    def __init__(self):
        #create message subscriber for receiving images
        #self.image_front = rospy.Subscriber('/ardrone/image_raw', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/ardrone/bottom/image_raw', Image, self.image_callback)
        #create message publisher for sending drone movement commands
        self.movement_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.state="SEARCHING" 
    def image_callback(self, image):
	image = ToOpenCV(image)
        location,magnitude = self.get_target_location(image)
        #
        #control logic/commands here
        #location will be a None object if target is not visible
        #otherwise, location is a tuple (x,y)
        #
        print location,magnitude
        print self.state
        twist = Twist() #create new empty twist message
        if self.state == "TRACKING":
            if(location is None):
		self.state = "SEARCHING"
		return
	    #go downward
	    #twist.linear.z = -0.1
	    #if(location[0]>177 and location[0]<183):
	#	print "find...."
	#	return
	    twist.linear.x = 0.1
	    if(location[0]<6 and location[0]>-6):
		print "desent"
		twist.linear.z = -3.0
	    elif(location[0]>10):
	        twist.angular.z = -0.002*location[0]
            elif(location[0]<10):
	        twist.angular.z=-0.002*location[0]
	    
	    if magnitude>21000000:
		self.state = "STOP"
                return
        #I'm just telling it to go straight up for now
	elif self.state=="SEARCHING":
            twist.linear.z = 0.1
	    twist.angular.z = -0.5
	    if location is not None:
		self.state = "TRACKING"
	elif self.state=="STOP":
	    #register callback function to land drone if we kill the program
	    rospy.on_shutdown(shutdown_callback) 

	    rospy.sleep(1) #wait for a second to wait for node to fully connect
	    drone_takeoff_pub.publish(Empty()) #command drone to takeoff
    	    print "done!"
	    return
        self.movement_pub.publish(twist) #publish message
    
    def get_target_location(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_orange = numpy.array([10,10,10])
        upper_orange = numpy.array([255,255,255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        M = cv2.moments(mask)
        location = None
        magnitude = 0
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            magnitude = M['m00']
            location = (cx-320, cy)
            cv2.circle(image, (cx,cy), 3, (0,0,255), -1)

        cv2.imshow("camera", image)
        cv2.waitKey(1)
        return location,magnitude




#callback function that gets run when node shuts down
def shutdown_callback():
    print "shutting down..."
    drone_land_pub.publish(Empty()) #make the drone land
    cv2.destroyAllWindows() #cleanup opencv windows
    print "done!"


if __name__ == "__main__":
    rospy.init_node("quadcopter_controller")
    #ardrone uses specialized topics for landing and taking off
    drone_takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    drone_land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
    controller = QuadcopterController()

    #register callback function to land drone if we kill the program
    rospy.on_shutdown(shutdown_callback) 

    rospy.sleep(1) #wait for a second to wait for node to fully connect
    drone_takeoff_pub.publish(Empty()) #command drone to takeoff

    #this function loops and waits for node to shutdown
    #all logic happens in the image_callback function
    rospy.spin()


