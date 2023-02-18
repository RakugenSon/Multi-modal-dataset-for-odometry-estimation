#coding:utf-8
import roslib
import rosbag
import rospy
import tf2_ros

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from nav_msgs.msg import Odometry



class ImageCreator():


    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag('rosbag.bag', 'r') as bag:   #要读取的bag文件；
            gt = open("rosbag.txt",'w')
            for topic,msg,t in bag.read_messages():
                if topic == "/ackermann_steering_controller/odom":  #imu topic；
                    pos_y = "%.6f" % msg.pose.pose.position.y
                    pos_x = "%.6f" % msg.pose.pose.position.x
                    pos_z = "%.6f" % msg.pose.pose.position.z
                    ori_w = "%.6f" % msg.pose.pose.orientation.w
                    ori_x = "%.6f" % msg.pose.pose.orientation.x
                    ori_y = "%.6f" % msg.pose.pose.orientation.y
                    ori_z = "%.6f" % msg.pose.pose.orientation.z
                    timegt = "%.6f" %  msg.header.stamp.to_sec()
                    gtdata = timegt + " " + pos_x + " " + pos_y + " " + pos_z + " " + ori_x + " " + ori_y + " " + ori_z + " " + ori_w
                    gt.write(gtdata)
                    gt.write('\n')
            gt.close()


if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass