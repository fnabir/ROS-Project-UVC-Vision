#!/usr/bin/env python
import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class TestVisionNode :
    def __init__(self):
        #create the cvbridge object. CvBridge is a method used
        #help convert data coming from image_raw/compressed topic into 
        #opencv data type
        #opencv data type is a 2D matrix with RGB values
        self.bridge = CvBridge()

        #rospy.Subscribe helps us subscribe to raw image/compressed topic.
        #When we subscribe to image/compressed topic
        #we established a connection in the TCP/IP network to
        #tap into the data streaming at the topic image_raw/compressed.
        #In ros the data are treated/manipulated with a callback function.
        #ROS uses a container to help exchange data from various nodes (special programs
        #that uses ROS data transportation technology) 
        #A callback function is a technique to handle data when it is available
        #this is why in the TestVisionNode class, the method "callback" has a 
        # try.. exception structure to make sure that the program does not terminate
        # when there is an incomplete data streaming in the ros network or ros pipeline
        # CompressedImage is a message template used in the topic image_raw/compressed.
        self.sub = rospy.Subscriber("image_raw/compressed", CompressedImage, self.callback)

    def callback(self, data):
        
        #the try .. exception structure enables this callback function
        #to reiterate itself if an error of missing data or incomplete data
        #are encountered when this ros node is subscribing to image_raw/compressed
        try:
            #convert the raw image (ros data object or message stored in a cache)
            #to opencv data type often in a form of 2D matrix
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
                print (e)
        #this will create a visualization of the image
        cv2.imshow("image",cv_image)
        cv2.waitkey(1) #refresh every second

def main():
    vn=TestVisionNode()
    #we initiate this codes as a node that uses the ROS 
    #transport technology. ROS uses TCP/IP protocol
    #as a pipeline to exchange data/commands between ros nodes
    #These data are bundled into containers called messages
    rospy.init_node("test_vision_node")
    try:
        #this will make sure the program will run in loop using the TestVisionNode.callback method
        rospy.spin()
    # When we keyed int ctrl-c, the program will print "Node Shutdown" and proceed to 
    # to the next line of the code which is cv2.closeAllWindows()
    except KeyboardInterrupt:
        print("Node Shutdown")
    cv2.closeAllWindows()
if __name__ == "__main__":
    main()
