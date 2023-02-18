#coding:utf-8
import roslib;
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class ImageCreator():


    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag('rosbag.bag', 'r') as bag:   #要读取的bag文件；
            imu = open("rosbag.txt",'w')
            for topic,msg,t in bag.read_messages():
                if topic == "/imu/data":  #imu topic；
                    acc_y = "%.6f" % msg.linear_acceleration.y
                    acc_x = "%.6f" % msg.linear_acceleration.x
                    acc_z = "%.6f" % msg.linear_acceleration.z
                    w_y = "%.6f" % msg.angular_velocity.y
                    w_x = "%.6f" % msg.angular_velocity.x
                    w_z = "%.6f" % msg.angular_velocity.z
                    timeimu = "%.6f" %  msg.header.stamp.to_sec()
                    imudata = timeimu + " " + w_x + " " + w_y + " " + w_z + " " + acc_x + " " + acc_y + " " + acc_z
                    imu.write(imudata)
                    imu.write('\n')
            imu.close()


if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
