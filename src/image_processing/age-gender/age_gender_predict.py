import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String

# OpenCV ile yüz, cinsiyet ve yaş tanıma için gerekli modellerin yolları
faceProto = "opencv_face_detector.pbtxt"
faceModel = "opencv_face_detector_uint8.pb"
ageProto = "age_deploy.prototxt"
ageModel = "age_net.caffemodel"
genderProto = "gender_deploy.prototxt"
genderModel = "gender_net.caffemodel"
faceNet = cv2.dnn.readNet(faceModel, faceProto)
ageNet = cv2.dnn.readNet(ageModel, ageProto)
genderNet = cv2.dnn.readNet(genderModel, genderProto)

MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
ageList = ['(0-3)', '(4-7)', '(8-12)', '(15-24)', '(25-32)', '(38-43)', '(48-59)', '(60-100)']
genderList = ['Male', 'Female']
# CvBridge nesnesi
bridge = CvBridge()

# Görüntüleri ve sonuçları yayınlamak için ROS publisher'larını oluştur
rospy.init_node('age_gender_detector', anonymous=True)
image_pub = rospy.Publisher('/detected_faces/compressed', CompressedImage, queue_size=10)
gender_pub = rospy.Publisher('/detected_gender', String, queue_size=10)
age_pub = rospy.Publisher('/detected_age', String, queue_size=10)

def getFaceBox(faceNet, frame):
    frameHeight = frame.shape[0]
    frameWidth = frame.shape[1]
    blob = cv2.dnn.blobFromImage(frame, 1.0, (227, 227), [104, 117, 123], swapRB=False)
    faceNet.setInput(blob)
    detection = faceNet.forward()
    faceBoxes = []
    for i in range(detection.shape[2]):
        confidence = detection[0, 0, i, 2]
        if confidence > 0.7:
            x1 = int(detection[0, 0, i, 3] * frameWidth)
            y1 = int(detection[0, 0, i, 4] * frameHeight)
            x2 = int(detection[0, 0, i, 5] * frameWidth)
            y2 = int(detection[0, 0, i, 6] * frameHeight)
            faceBoxes.append([x1, y1, x2, y2])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
    return frame, faceBoxes

def publish_data(frame, faceBoxes):
    for faceBox in faceBoxes:
        face = frame[max(0, faceBox[1]):min(faceBox[3], frame.shape[0] - 1),
                     max(0, faceBox[0]):min(faceBox[2], frame.shape[1] - 1)]
        blob = cv2.dnn.blobFromImage(face, 1.0, (227, 227), MODEL_MEAN_VALUES, swapRB=False)
        
        # Cinsiyet tahmini
        genderNet.setInput(blob)
        genderPred = genderNet.forward()
        gender = genderList[genderPred[0].argmax()]
        
        # Yaş tahmini
        ageNet.setInput(blob)
        agePred = ageNet.forward()
        age = ageList[agePred[0].argmax()]

        # Cinsiyet ve yaş bilgisini görüntü üzerine yaz
        label = "{}:{}".format(gender, age)
        cv2.putText(frame, label, (faceBox[0], faceBox[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

    # Görüntüyü sıkıştırılmış ROS mesajına dönüştür ve yayınla
    try:
        image_pub.publish(bridge.cv2_to_compressed_imgmsg(frame, "jpg"))
    except CvBridgeError as e:
        print(e)

def image_callback(msg):
    try:
        # ROS Image mesajını OpenCV görüntüsüne dönüştür
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Görüntü üzerinde yüz tanıma ve cinsiyet/yaş tahmini yap
    frame, faceBoxes = getFaceBox(faceNet, cv_image)
    if faceBoxes:
        publish_data(frame, faceBoxes)
    else:
        print("No face detected")

# '/usb_cam/image_raw' topiğine abone ol
sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

# ROS'u sürekli dinlemeye al, ctrl+c ile durdurulana kadar çalışmaya devam et
rospy.spin()

