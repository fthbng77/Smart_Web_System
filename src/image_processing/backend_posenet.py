#!/usr/bin/env python3
from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from jetson_inference import poseNet
from jetson_utils import cudaFromNumpy
import cv2
import numpy as np

app = Flask(__name__)
CORS(app)

net = None
pub = None
cv_img = None
compressed_pub = None
model_thread = None

def draw_pose_on_image(image, pose):
    color = (0, 255, 0)  # Green color for drawing
    thickness = 2  # Line thickness

    for link in pose.Links:
        keypoint_a = pose.Keypoints[link[0]]
        keypoint_b = pose.Keypoints[link[1]]

        # Check if keypoints are valid by checking their coordinates
        if keypoint_a.x >= 0 and keypoint_a.y >= 0 and keypoint_b.x >= 0 and keypoint_b.y >= 0:
            start_point = (int(keypoint_a.x), int(keypoint_a.y))
            end_point = (int(keypoint_b.x), int(keypoint_b.y))
            cv2.line(image, start_point, end_point, color, thickness)

def image_callback(msg):
    global cv_img
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)

    cuda_img = cudaFromNumpy(cv_img)

    # Perform pose estimation
    poses = net.Process(cuda_img)

    # Draw each detected pose on the image
    for pose in poses:
        draw_pose_on_image(cv_img, pose)

    # Convert the image back to BGR color space for OpenCV compatibility
    cv_img_bgr = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)

    # Encode the modified image to a buffer, defaulting to JPEG format
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    success, encoded_image = cv2.imencode('.jpg', cv_img_bgr, encode_param)
    if success:
        ros_compressed_img = CompressedImage()
        ros_compressed_img.header.stamp = rospy.Time.now()
        ros_compressed_img.format = "jpeg"
        ros_compressed_img.data = np.array(encoded_image).tobytes()
        compressed_pub.publish(ros_compressed_img)
    else:
        rospy.logerr("Image encoding failed")

def run_ai_model():
    global net, pub, cv_img, compressed_pub
    if not rospy.core.is_initialized():
        rospy.init_node('posenet_node', anonymous=True, disable_signals=True)

    net = poseNet('resnet18-body')
    pub = rospy.Publisher('/pose_result', String, queue_size=10)
    compressed_pub = rospy.Publisher('/PoseNet/compressed', CompressedImage, queue_size=10)
    rospy.Subscriber('/webcam/image_raw', Image, image_callback)
    rospy.spin()

@app.route('/start-ai-model', methods=['POST'])
def start_ai_model():
    global model_thread
    if model_thread is None or not model_thread.is_alive():
        model_thread = threading.Thread(target=run_ai_model)
        model_thread.start()
        response_message = {"message": "AI model is starting."}
    else:
        response_message = {"message": "AI model is already running."}
    return jsonify(response_message)

@app.route('/<path:path>', methods=['GET', 'POST', 'PUT', 'DELETE'])
def catch_all(path):
    app.logger.warning(f"Received unexpected request to /{path}")
    return jsonify({"error": "Endpoint not found"}), 404


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)