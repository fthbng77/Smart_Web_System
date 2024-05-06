#!/usr/bin/env python3

from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, CompressedImage
from jetson_inference import segNet
from jetson_utils import videoOutput, cudaOverlay, cudaDeviceSynchronize
from segnet_utils import *
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from jetson_utils import cudaFromNumpy, cudaToNumpy
import cv2
app = Flask(__name__)
CORS(app)

class Args:
    def __init__(self):
        self.stats = False
        self.visualize = "overlay"
        self.alpha = 150
        self.mask = None

args = Args()

network = "fcn-resnet18-deepscene-576x320"
alpha = 150  # 0 to 255
output_path = "output.mp4"
model_thread = None
pub = None  # Declare the publisher globally

def run_ai_model():
    global net, pub, img_format, output, buffers, bridge
    if not rospy.core.is_initialized():
        rospy.init_node('gokmen_node', anonymous=True, disable_signals=True)

    net = segNet(network)
    net.SetOverlayAlpha(alpha)
    output = videoOutput(output_path)

    buffers = segmentationBuffers(net, args)
    img_format = None
    bridge = CvBridge()
    pub = rospy.Publisher('/SegNet/compressed', CompressedImage, queue_size=10)  # Define the publisher for /SegNet/compressed
    
    def image_callback(data):
        global img_format
        try:
            img_input = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            if img_format is None:
                img_format = data.encoding
                buffers.Alloc(img_input.shape, img_format)
            else:
                img_input = cudaFromNumpy(img_input)
            net.Process(img_input)
            if buffers.overlay:
                net.Overlay(buffers.overlay)
            if buffers.mask:
                net.Mask(buffers.mask)
            if buffers.composite:
                cudaOverlay(buffers.overlay, buffers.composite, 0, 0)
                cudaOverlay(buffers.mask, buffers.composite, buffers.overlay.width, 0)
            output.Render(buffers.output)
            output.SetStatus("{:s} | Network {:.0f} FPS".format(network, net.GetNetworkFPS()))
            cudaDeviceSynchronize()
            net.PrintProfilerTimes()
            buffers.ComputeStats()
            numpy_output = cudaToNumpy(buffers.output)
            numpy_output = cv2.cvtColor(numpy_output, cv2.COLOR_BGR2RGB)
            
            # Convert the output to a compressed image and publish to /SegNet/compressed
            compressed_img = bridge.cv2_to_compressed_imgmsg(numpy_output, dst_format='png')
            pub.publish(compressed_img)
            
        except CvBridgeError as e:
            print(e)
    
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
