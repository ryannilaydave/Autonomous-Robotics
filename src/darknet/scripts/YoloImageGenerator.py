#!/usr/bin/env python
import rospy, cv2, cv_bridge
import numpy as np
import getpass
from sensor_msgs.msg import Image

class YoloImageGenerator:
    def __init__(self   ):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        self.counter = 232

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        (h, w) = image.shape[:2]
        imageRatio = 4
        image = cv2.resize(image, (w/imageRatio,h/imageRatio))
        cv2.imshow("Test", image)
        filepath='/home/' + getpass.getuser() + '/catkin_ws/src/comp4034/src/assignment/darknet/images/'+ getpass.getuser() + "_" + str(self.counter) + '.jpg'
        cv2.imwrite(filepath, image)
        self.writeFile()
        print("Written to : " + filepath)
        cv2.waitKey(1500)
        self.counter+=1

    def readFile(self):
        f = open("counter.txt", "r")
        if(not f.read()) : return 0; return int(f.read())

    def writeFile(self):
        f = open("counter.txt", "w")
        f.write(str(self.counter))
        f.close()

rospy.init_node('yoloImageGenerator')
generator = YoloImageGenerator()
rospy.spin()