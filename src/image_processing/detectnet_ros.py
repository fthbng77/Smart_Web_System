#!/usr/bin/env python3
from jetson_inference import detectNet
import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from jetson_utils import cudaFromNumpy
from cv_bridge import CvBridge
import cv2
import numpy as np

def image_callback(msg):
    global cv_img
    # CompressedImage mesajını numpy array'ine çevir
    np_arr = np.fromstring(msg.data, np.uint8)
    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    cuda_img = cudaFromNumpy(cv_img)

    detections = net.Detect(cuda_img)

    # For each detection, draw a bounding box and publish the result
    for detection in detections:
        class_desc = net.GetClassDesc(detection.ClassID)
        confidence = detection.Confidence
        bbox = (detection.Left, detection.Top, detection.Right, detection.Bottom)
        cv2.rectangle(cv_img, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 0), 1)
        text = "{}: {:.2f}%".format(class_desc, confidence * 100)
        cv2.putText(cv_img, text, (int(bbox[0]), int(bbox[1])-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # Publish the result
        pub.publish(class_desc)

        rospy.loginfo("Detected '{}' (class #{}) with {:.2f}% confidence".format(class_desc, detection.ClassID, confidence * 100))

    # publish compressed image
    compressed_img_msg = CvBridge().cv2_to_compressed_imgmsg(cv_img, dst_format='jpeg')
    compressed_pub.publish(compressed_img_msg)

def main():
    global net, pub, cv_img, compressed_pub

    net = detectNet("ssd-mobilenet-v2")

    rospy.init_node('detectnet_node', anonymous=True)

    # Abone olurken mesaj tipini CompressedImage olarak değiştir
    sub = rospy.Subscriber('/webcam/image_raw/compressed', CompressedImage, image_callback)

    pub = rospy.Publisher('/detectnet_result', String, queue_size=10)
    compressed_pub = rospy.Publisher('/detect/image_compressed', CompressedImage, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    cv_img = None
    main()
