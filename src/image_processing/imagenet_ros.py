#!/usr/bin/env python3
from jetson_inference import imageNet
import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from jetson_utils import cudaFromNumpy
from cv_bridge import CvBridge
import cv2
import numpy as np

def image_callback(msg):
    global cv_img
    cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    cuda_img = cudaFromNumpy(cv_img)

    # classify the image
    class_idx, confidence = net.Classify(cuda_img)

    # find the object description
    class_desc = net.GetClassDesc(class_idx)

    # publish the result
    pub.publish(class_desc)

    # display the result on the image window
    font = cv2.FONT_HERSHEY_SIMPLEX
    text = "{}: {:.2f}%".format(class_desc, confidence * 100)
    cv2.putText(cv_img, text, (10, 30), font, 1, (255, 255, 255), 2)

    rospy.loginfo("image is recognized as '{:s}' (class #{:d}) with {:f}% confidence".format(class_desc, class_idx, confidence * 100))

    # publish compressed image
    compressed_img_msg = CvBridge().cv2_to_compressed_imgmsg(cv_img)
    compressed_pub.publish(compressed_img_msg)

def main():
    global net, pub, cv_img, compressed_pub

    net = imageNet("resnet-18")

    rospy.init_node('imagenet_node', anonymous=True)

    sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    pub = rospy.Publisher('/imagenet_result', String, queue_size=10)
    compressed_pub = rospy.Publisher('/usb_cam/image_compressedd', CompressedImage, queue_size=10)

    # create a window to display the image
    rospy.spin()

if __name__ == '__main__':
    cv_img = None
    main()

