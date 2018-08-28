# -*- coding: utf-8 -*-
"""
Created on Mon Feb 26 11:30:10 2018

@author: Nicholas Venis.
"""
import rospy, cv2, cv_bridge, numpy, time
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge() #bridges are used to convert ROS images such as from the robot to OpenCV images that we can operate on
    cv2.namedWindow("window", 1) #creates a new window to view the robot camera feed
    self.image_sub = rospy.Subscriber('turtlebot/camera/rgb/image_raw', Image, self.image_callback) #create an image subscriber to retrieve image data from the turtlebot
                                      
    self.laser_sub = rospy.Subscriber('turtlebot/scan', LaserScan, self.laser_callback) #retrieve laser data by subscribing to the laser topic, then calling it in the laser callback method to provide continuous data
                                      
    self.cmd_vel_pub = rospy.Publisher('turtlebot/cmd_vel_mux/input/teleop',Twist, queue_size=1) #create a new publisher to send twist messages, this allows for movement

    #global variables initialised in init for use further in the routine
    self.colourFound = False
    self.stopped = False
    self.rFound = False
    self.yFound = False
    self.gFound = False
    self.bFound = False
    self.canMove = False
    self.objectCount = 0

                                                                                          
    self.twist = Twist() #Define what a twist message is
    
    self.laser = None #initialise the laser variable so it starts on runtime

  #the laser callback function provides a continuous stream of laser that is stored in the variable self.laser
  def laser_callback (self, data):
    self.laser = data

  #the image callback function provides the routine with continuous image data and is where the processing of subroutines takes place
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8') #convert the image from a matrix to a workable image in BGR format
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #converts the image from BGR to HSV for easier masking
    
    maskR = cv2.inRange(hsv,numpy.array([ 0,  100,  50]),numpy.array([5, 255, 255])) #the mask for finding red defined as being between two intensity ranges in HSV, if a colour meets this criteria it is masked
    maskG = cv2.inRange(hsv,numpy.array([ 60,  100,  50]),numpy.array([70, 255, 255])) #the mask for finding green defined as being between two intensity ranges in HSV, if a colour meets this criteria it is masked
    maskB = cv2.inRange(hsv,numpy.array([ 110,  100,  50]),numpy.array([130, 255, 255])) #the mask for finding blue defined as being between two intensity ranges in HSV, if a colour meets this criteria it is masked
    maskY = cv2.inRange(hsv,numpy.array([ 25,  100,  50]),numpy.array([30, 255, 255])) #the mask for finding yellow defined as being between two intensity ranges in HSV, if a colour meets this criteria it is masked

    
    laserData = self.laser.ranges #laserData stores the 32bit Vector of laser data for processing
    middleData = int(round(len(laserData)-1)/2) #get middle of laser data. get the length of range data and find middle value. Round as decimal as is not used in processing
    leftData = laserData[middleData-10:middleData]#get all values indexed 10 from the centre to the centre
    rightData = laserData[middleData:middleData+10] #get the values indexed 10 to the right of the centre of the range data
   
    
    h, w, d = image.shape #get the dimensions of the camera feed
    search_top = 1*h/4 #restrict to the top quarter of the image
    search_bot = 3*h/4 #restrict to the bottom quarter of the image
#    mask[0:search_top, 0:w] = 0
#    mask[search_bot:h, 0:w] = 0
#    M = cv2.moments(mask)


    if self.objectCount <= 4: #an end goal for the subroutine, completes when all colours found
            if self.rFound == False and self.objectCount == 0: # only runs if red hasn't been found and if it is the first colour
                maskR[0:search_top, 0:w] = 0 #mask using the defined red mask to filter for the red pillar, minus top quarter
                maskR[search_bot:h, 0:w] = 0 #mask using the defined red mask to filter for the red pillar, minus bottom quarter
                M = cv2.moments(maskR) #Define M as the masked image
                if M['m00'] > 0 and M['m00']<9000000: # if the area of the detected shape in mask is greater than 0 and less than a preset number, this makes the robot stop within 1 metre
                  cx = int(M['m10']/M['m00'])#get centre coordinates
                  cy = int(M['m01']/M['m00'])#get centre coordinates
                  cv2.circle(image, (cx, cy), 20, (0,0,255), -1)#draw a red circle to follow red shape
                  # BEGIN CONTROL
                  self.colourFound = True #engage colour lock
                  print("Object found, moving towards")#publish a message a colour is found
                  err = cx - w/2#calcualte the difference between the center of the masked shape and centre of the image
                  self.twist.linear.x = 0.3 #move forward at speed 0.3
                  self.twist.angular.z = -float(err) / 200 #move angular based on the difference between masked shape and centre of the image
                  self.cmd_vel_pub.publish(self.twist) #publish the twist message and move the robot
                if M["m00"] > 9000000: #if shape area is greater and therefore the robot has arrived at it
                  print("Red found!") #red found
                  self.rFound = True #set the red boolean to true to prevent it running again
                  self.objectCount = self.objectCount + 1 #add a count to the object
                  twistMsg = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,2.5)) #turn 90 degrees to the left
                  self.cmd_vel_pub.publish(twistMsg) #move
                  time.sleep(2)#sleep to demonstrate position of the robot
                  self.colourFound = False #disengage colour lock

            if self.yFound == False and self.objectCount == 1: # only runs if yellow hasn't been found and if it is the second colour
                maskY[0:search_top, 0:w] = 0 #masks the same as previous however uses yellow mask
                maskY[search_bot:h, 0:w] = 0
                M = cv2.moments(maskY)
                #function is the same as previous
                if M['m00'] > 0 and M['m00']<9000000:
                  cx = int(M['m10']/M['m00'])
                  cy = int(M['m01']/M['m00'])
                  cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                  # BEGIN CONTROL
                  self.colourFound = True
                  print("Object found, moving towards")
                  err = cx - w/2
                  self.twist.linear.x = 0.3
                  self.twist.angular.z = -float(err) / 200
                  self.cmd_vel_pub.publish(self.twist)
                if M["m00"] > 9000000:
                  print("Yellow found!")#publish a message that yellow is found
                  self.rFound = True
                  self.objectCount = self.objectCount + 1
                  twistMsg = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,2.5))
                  self.cmd_vel_pub.publish(twistMsg)
                  time.sleep(2)
                  self.colourFound = False  

            if self.gFound == False and self.objectCount == 2:# only runs if green hasn't been found and if it is the third colour
                maskG[0:search_top, 0:w] = 0 #masks the same as previous however uses yellow mask
                maskG[search_bot:h, 0:w] = 0
                M = cv2.moments(maskG)
                #same as previous funtion
                if M['m00'] > 0 and M['m00']<9000000:
                  cx = int(M['m10']/M['m00'])
                  cy = int(M['m01']/M['m00'])
                  cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                  # BEGIN CONTROL
                  self.colourFound = True
                  print("Object found, moving towards")
                  err = cx - w/2
                  self.twist.linear.x = 0.3
                  self.twist.angular.z = -float(err) / 200
                  self.cmd_vel_pub.publish(self.twist)
                if M["m00"] > 9000000:
                  print("Green found!")#publish a message that yellow is found
                  self.rFound = True
                  self.objectCount = self.objectCount + 1
                  twistMsg = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,2.5))
                  self.cmd_vel_pub.publish(twistMsg)
                  time.sleep(2)
                  self.colourFound = False  

            if self.bFound == False and self.objectCount == 3:# only runs if blue hasn't been found and if it is the fourth colour
                maskB[0:search_top, 0:w] = 0 #masks the same as previous however uses blue mask
                maskB[search_bot:h, 0:w] = 0
                M = cv2.moments(maskB)
                #same as previous function
                if M['m00'] > 0 and M['m00']<9000000:
                  cx = int(M['m10']/M['m00'])
                  cy = int(M['m01']/M['m00'])
                  cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                  # BEGIN CONTROL
                  self.colourFound = True
                  print("Object found, moving towards")
                  err = cx - w/2
                  self.twist.linear.x = 0.3
                  self.twist.angular.z = -float(err) / 200
                  self.cmd_vel_pub.publish(self.twist)
                if M["m00"] > 9000000:
                  print("Blue found!")#sends a message blue is found
                  self.rFound = True
                  self.objectCount = self.objectCount + 1
                  twistMsg = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,2.5))
                  self.cmd_vel_pub.publish(twistMsg)
                  time.sleep(2)
                  self.colourFound = False    

           #roaming code
            if self.colourFound == False:#if colour lock is not on
                if numpy.nanmin(leftData) > 2 and numpy.nanmin(rightData) > 2:#there is space either side of robot
                    twistMsg = Twist(Vector3(0.5,0.0,0.0), Vector3(0.0,0.0,0.0))#the robot is free to move forward
                    self.cmd_vel_pub.publish(twistMsg)
                elif numpy.nanmin(leftData) < 2: # if there is no space on the left side of the robot correct this to follow the left wall
                    twistMsg = Twist(Vector3(0.2,0.0,0.0), Vector3(0.0,0.0,0.5))#the robot moves forward and left
                    self.cmd_vel_pub.publish(twistMsg)
                elif  numpy.nanmin(laserData) <  0.75 or numpy.min(laserData) == None: #if an of the values return as being below a threshold or the robot collides, stop and turn around
                    twistMsg = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,-0.5))
                    self.cmd_vel_pub.publish(twistMsg)
         

        
    

    cv2.imshow("window", image) #show the window created earlier
    cv2.waitKey(3)

      
    

rospy.init_node('follower') #intialise the class
follower = Follower()#call the class
rospy.spin() #keeps python from exiting until node is finished
# END ALL
