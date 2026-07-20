#!/usr/bin/env python


import os
import rospy
from target_recognition.srv import ProcessImage, ProcessImageResponse
from sensor_msgs.msg import Image
import cv2
import numpy as np
import onnxruntime
from cv_bridge import CvBridge, CvBridgeError
from skimage.restoration import wiener

bridge = CvBridge()

def detect_blur_fft(image, blur_threshold):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    f = np.fft.fft2(gray)
    fshift = np.fft.fftshift(f)
    magnitude_spectrum = 20 * np.log(np.abs(fshift))
    rows, cols = gray.shape
    crow, ccol = rows // 2 , cols // 2
    high_freq = np.sum(np.abs(magnitude_spectrum[crow-30:crow+30, ccol-30:ccol+30]))
    total = np.sum(np.abs(magnitude_spectrum))
    ratio = high_freq / total
    if ratio < blur_threshold:
        return True, ratio 
    else:
        return False, ratio 

def motion_blur_psf(length, angle):
    psf = np.zeros((length, length))
    center = length // 2
    slope = np.tan(np.deg2rad(angle))
    for i in range(length):
        offset = int(slope * (i - center))
        psf[center + offset, i] = 1
    return psf / psf.sum()

def wiener_deblur(blurred_image, psf, K=0.01):
    return wiener(blurred_image, psf, K)

def preprocess_image_opencv(cv_image):
    image = cv2.resize(cv_image, (28, 28))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = image.astype(np.float32)
    image = np.expand_dims(image, axis=-1)
    image = (image / 255.0 - 0.5) / 0.5
    image = np.expand_dims(image, axis=0)
    image = np.transpose(image, (0, 3, 1, 2))
    return image

def handle_process_image(req):
    try:
        script_directory = os.path.dirname(os.path.abspath(__file__))
        onnx_model_path = os.path.dirname(script_directory)
        onnx_model_path = onnx_model_path + "/classifier.onnx"
        blur_threshold = rospy.get_param('~blur_threshold', 0.2)
        cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")
        is_blurred, ratio = detect_blur_fft(cv_image, blur_threshold)
        #rospy.loginfo("Laplacian variance: %f", ratio)
        if is_blurred:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            cv_image = cv_image / 255.0 
            psf = motion_blur_psf(length=5, angle=3)
            cv_image = wiener_deblur(cv_image, psf)
            cv_image = (cv_image * 255).astype(np.uint8)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        net_session = onnxruntime.InferenceSession(onnx_model_path)
        inputs = {net_session.get_inputs()[0].name: preprocess_image_opencv(cv_image)}
        outs = net_session.run(None, inputs)[0]

        #rospy.loginfo("onnx weights: %s", outs)
        prediction = outs.argmax(axis=1)[0] + 1
        weight = np.max(outs)
        #rospy.loginfo("onnx prediction: %d", prediction)

        return ProcessImageResponse(weight, prediction)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)
        return ProcessImageResponse(-1)
    except Exception as e:
        rospy.logerr("Error processing image: %s", e)
        return ProcessImageResponse(-1)

def target_recognition_node():
    rospy.init_node('target_recognition_node')
    s = rospy.Service('process_image', ProcessImage, handle_process_image)
    rospy.loginfo("Ready to process images.")
    rospy.spin()

if __name__ == "__main__":
    target_recognition_node()

