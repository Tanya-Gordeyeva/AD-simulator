#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage, Image
import rospy

class DemoNode(object):
    def __init__(self):
        self.node_name = "LineDetectorNode"
        self.sub_image = rospy.Subscriber("/None/corrected_image/compressed", CompressedImage, self.cbImage, queue_size=1)
        self.pub_cmd = rospy.Publisher("/None/car_cmd", Twist2DStamped, queue_size=1)


    def cbImage(self, image_msg):
        msg = Twist2DStamped()
        msg.v = 0.1
        msg.omega = 0.0
        self.pub_cmd.publish(msg)

if __name__ == '__main__': 
    rospy.init_node('demo',anonymous=False)
    demo_node = DemoNode()
    rospy.spin()
