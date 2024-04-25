import time
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy, cudaToNumpy
import cv2
import numpy as np

def process_image(image_path):
    # Görüntüyü yükle ve uygun formata dönüştür
    cv_img = cv2.imread(image_path)
    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    cuda_img = cudaFromNumpy(cv_img)

    # Nesne tespiti ağı yükle
    net = detectNet("ssd-mobilenet-v2")

    # Görüntü üzerinde nesne tespiti yapmak için zamanı ölç
    start_time = time.time()  # Zaman ölçümünü başlat
    detections = net.Detect(cuda_img)
    end_time = time.time()  # Zaman ölçümünü bitir
    fps = 1 / (end_time - start_time)  # FPS hesapla

    # Tespit edilen her nesne için sınırlayıcı kutu çiz
    for detection in detections:
        class_desc = net.GetClassDesc(detection.ClassID)
        confidence = detection.Confidence
        bbox = (detection.Left, detection.Top, detection.Right, detection.Bottom)
        cv2.rectangle(cv_img, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 0), 2)
        text = "{}: {:.2f}%".format(class_desc, confidence * 100)
        cv2.putText(cv_img, text, (int(bbox[0]), int(bbox[1])-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # FPS değerini görüntüye ekle
    cv2.putText(cv_img, "FPS: {:.2f}".format(fps), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 2)

    # Sonuçları göster
    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
    cv2.imshow('Detected Objects', cv_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    process_image('unnamed.jpg')

