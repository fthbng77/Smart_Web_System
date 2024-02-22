from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy

app = Flask(__name__)
CORS(app)

net = None
pub = None
cv_img = None
compressed_pub = None
model_thread = None

def image_callback(msg):
    global cv_img, net, pub, compressed_pub
    try:
        cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except CvBridgeError as e:
        print(e)

    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    cuda_img = cudaFromNumpy(cv_img)
    detections = net.Detect(cuda_img)

    detections = net.Detect(cuda_img)
    for detection in detections:
        class_desc = net.GetClassDesc(detection.ClassID)
        confidence = detection.Confidence
        bbox = (detection.Left, detection.Top, detection.Right, detection.Bottom)
        cv2.rectangle(cv_img, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 0), 1)
        text = f"{class_desc}: {confidence * 100:.2f}%"
        cv2.putText(cv_img, text, (int(bbox[0]), int(bbox[1])-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        rospy.loginfo(f"Detected '{class_desc}' with {confidence * 100:.2f}% confidence")

    compressed_img_msg = CvBridge().cv2_to_compressed_imgmsg(cv_img, dst_format='jpeg')
    compressed_pub.publish(compressed_img_msg)

def run_ai_model():
    global net, pub, cv_img, compressed_pub
    if not rospy.core.is_initialized():
        rospy.init_node('detectnet_node', anonymous=True, disable_signals=True)

    net = detectNet("facenet")
    pub = rospy.Publisher('/facenet_result', String, queue_size=10)
    compressed_pub = rospy.Publisher('/FaceNet/compressed', CompressedImage, queue_size=10)
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
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