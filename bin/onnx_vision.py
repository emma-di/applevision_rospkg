#!/usr/bin/python3
# don't use this, can trash it

import cv2
from yolov8 import YOLOv8
import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
from applevision_rospkg.msg import RegionOfInterestWithConfidenceStamped
from helpers import HeaderCalc
from applevision_vision import MODEL_PATH

CONFIDENCE_THRESH = 0.75

class AppleVisionHandler:

    def __init__(self, net, topic: str, frame: str,
                 debug_frame_topic: str) -> None:
        self.net = net
        self.pub = rospy.Publisher(topic,
                                   RegionOfInterestWithConfidenceStamped,
                                   queue_size=10)
        self.pub_debug = rospy.Publisher(debug_frame_topic,
                                         Image,
                                         queue_size=10)
        self._br = CvBridge()
        self._header = HeaderCalc(frame)

    def run_applevision(self, im: Image):
        yolov8_detector = YOLOv8(MODEL_PATH, conf_thres=0.75, iou_thres=0.5)
        
        if rospy.Time.now() - im.header.stamp > rospy.Duration.from_sec(0.1):
            # this frame is too old
            rospy.logwarn_throttle_identical(3, 'CV: Ignoring old frame')
            return None
        
        frame = self._br.imgmsg_to_cv2(im, 'bgr8')
        cv2.imshow(frame)

        cv2.namedWindow("Detected Objects", cv2.WINDOW_NORMAL)
        while cap.isOpened():

            # Read frame from the video
            ret, frame = cap.read()

            if not ret:
                break

            # Update object localizer
            boxes, scores, class_ids = yolov8_detector(frame)
            rospy.logwarn(boxes)
            rospy.logwarn(scores)
            rospy.logwarn(class_ids)

            combined_img = yolov8_detector.draw_detections(frame)
            cv2.imshow("Detected Objects", combined_img)

            # Press key q to stop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        debug_im = self._br.cv2_to_imgmsg(frame)
        self.pub_debug.publish(debug_im)
        

if __name__ == '__main__':
    rospy.init_node('applevision_fake_sensor_data')

    net = cv2.dnn.readNet(str(MODEL_PATH))
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

    # TODO: add image_proc
    vision_handler = AppleVisionHandler(net,
                                        'applevision/apple_camera',
                                        'palm_camera',
                                        'applevision/debug_apple_camera')
    sub = rospy.Subscriber('palm_camera/image_rect_color',
                           Image,
                           vision_handler.run_applevision,
                           queue_size=20)

    rospy.spin()


# Initialize the webcam
video_path = "test-materials/rleaves.MOV"
cap = cv2.VideoCapture(0) #VideoCapture(0) for webcam, (video_path) for video

# Initialize YOLOv8 object detector
model_path = "best-roboflow.onnx"
yolov8_detector = YOLOv8(model_path, conf_thres=0.75, iou_thres=0.5)

cv2.namedWindow("Detected Objects", cv2.WINDOW_NORMAL)
while cap.isOpened():

    # Read frame from the video
    ret, frame = cap.read()

    if not ret:
        break

    # Update object localizer
    boxes, scores, class_ids = yolov8_detector(frame)

    combined_img = yolov8_detector.draw_detections(frame)
    cv2.imshow("Detected Objects", combined_img)

    # Press key q to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break