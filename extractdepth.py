#coding:utf-8

import rosbag
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

path='' #存放图片的位置
class ImageCreator():


   def __init__(self):
       self.bridge = CvBridge()
       with rosbag.Bag('rosbag.bag', 'r') as bag:
           #要读取的bag文件；
           for topic,msg,t in bag.read_messages():
               if topic == "/d435/depth/image_raw":  #图像的topic；
                       try:
                           cv_image = self.bridge.imgmsg_to_cv2(msg,"16UC1")
                       except CvBridgeError as e:
                           print (e)
                       timestr = "%.6f" %  msg.header.stamp.to_sec()
                       #%.6f表示小数点后带有6位，可根据精确度需要修改；
                       image_name = timestr+ ".png" #图像命名：时间戳.jpg
                       cv2.imwrite(path+image_name, cv_image)  #保存；


if __name__ == '__main__':
   try:
       image_creator = ImageCreator()
   except rospy.ROSInterruptException:
       pass

