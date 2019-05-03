#!/usr/bin/env python
import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class TestVisionNode :
    def __init__(self):
        #initialize node (name)
        #rospy.init_node("test_vision_node")
        #create the cvbridge object
        self.bridge = CvBridge()
        #subscribe to raw image/compressed topic
        self.sub = rospy.Subscriber("image_raw/compressed", CompressedImage, self.callback)

    def callback(self, data):
        try:
            #convert the raw to opencv
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
                print (e)
    #display image
        cv2.imshow("image",cv_image)
        cv2.waitkey(1) #refresh every second

def main():
    vn=TestVisionNode()
    rospy.init_node("test_vision_node")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Node Shutdown")
    cv2.closeAllWindows()
if __name__ == "__main__":
    main()
