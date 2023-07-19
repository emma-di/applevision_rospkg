#!/usr/bin/python3

import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
from applevision_rospkg.msg import RegionOfInterestWithConfidenceStamped
from helpers import HeaderCalc
from applevision_vision import MODEL_PATH
from ultralytics import YOLO


CONFIDENCE_THRESH = 0.8
model = YOLO('/root/catkin_ws/src/applevision_rospkg/src/applevision_vision/best-roboflow.pt')

def format_yolov5(
    source
):  #Function taken from medium: https://medium.com/mlearning-ai/detecting-objects-with-yolov5-opencv-python-and-c-c7cf13d1483c
    # YOLOV5 needs a square image. Basically, expanding the image to a square
    # put the image in square big enough
    col, row, _ = source.shape
    _max = max(col, row)
    resized = np.zeros((_max, _max, 3), np.uint8)
    resized[0:col, 0:row] = source
    result=cv2.resize(source, (640,640))
    # resize to 640x640, normalize to [0,1[ and swap Red and Blue channels
    # result = cv2.dnn.blobFromImage(resized, 1 / 255.0, (640, 640), swapRB=True)
    return result


def unwrap_detection(input_image, output_data):
    class_ids = []
    confidences = []
    boxes = []

    rows = output_data.shape[0]
    image_width, image_height, _ = input_image.shape

    x_factor = image_width / 640
    y_factor = image_height / 360

    for r in range(rows):
        row = output_data[r] #[0] #initially row = output_data[r]
        rospy.logwarn(row)
        confidence = row[5]
        rospy.logwarn(confidence)
        if confidence >= CONFIDENCE_THRESH: #originally confidence >= THRESH... but there are multiple objects

            classes_scores = row[5:]
            _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
            class_id = max_indx[1]
            if (classes_scores[class_id] > .25):

                confidences.append(confidence)
                class_ids.append(class_id)

                x, y, w, h = row[0].item(), row[1].item(), row[2].item(
                ), row[3].item()
                left = int((x - 0.5 * w) * x_factor)
                top = int((y - 0.5 * h) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                box = np.array([left, top, width, height])
                # box = np.array([int(x), int(y), int(w), int(h)])
                boxes.append(box)
    rospy.logwarn(max(confidences))
    return class_ids, confidences, boxes


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
        if rospy.Time.now() - im.header.stamp > rospy.Duration.from_sec(0.1):
            # this frame is too old
            rospy.logwarn_throttle_identical(3, 'CV: Ignoring old frame')
            return None
        start = time.time()
        
        CLASESS_YOLO = ['Green Apple', 'Red Apple']
        frame = self._br.imgmsg_to_cv2(im, 'bgr8')
        adjusted_image = format_yolov5(frame)
        blob = cv2.dnn.blobFromImage(adjusted_image, 1/255.0, (640, 640), swapRB=True, crop=False)
        net.setInput(blob)
        preds = net.forward()
        
        class_ids, confs, boxes = list(), list(), list()

        image_height, image_width, _ = frame.shape
        x_factor = image_width / 640
        # rospy.logwarn(x_factor)
        y_factor = image_height / 360
        # x_factor = 10
        # y_factor = 5

        rows = preds[0].shape[0]

        for i in range(rows):
            row = preds[0][i]
            conf = row[4]
            
            classes_score = row[4:]
            _,_,_, max_idx = cv2.minMaxLoc(classes_score)
            class_id = max_idx[0]
            if (classes_score[class_id] > .25) and conf>CONFIDENCE_THRESH:
                confs.append(conf)
                label = CLASESS_YOLO[int(class_id)]
                class_ids.append(label)
                
                #extract boxes
                x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                left = int((x - 0.5 * w) * x_factor)
                top = int((y - 0.5 * h) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                box = np.array([left, top, width, height])
                boxes.append(box)
                
        r_class_ids, r_confs, r_boxes = list(), list(), list()

        indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.25, 0.45) 
        for i in indexes:
            r_class_ids.append(class_ids[i])
            r_confs.append(confs[i])
            r_boxes.append(boxes[i])
            
        for i in indexes:
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            
            cv2.rectangle(frame, (left, top), (left + width, top + height), (0,255,0), 3)
        # Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            cv2.putText(frame, f"Apple: {conf:.2}", (box[0] + 5, box[1] - 5),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        
        else:
            msg = RegionOfInterestWithConfidenceStamped(
                header=self._header.get_header(),
                x=0,
                y=0,
                w=0,
                h=0,
                image_w=640,
                image_h=360,
                confidence=0)
            self.pub.publish(msg)

        debug_im = self._br.cv2_to_imgmsg(frame)
        self.pub_debug.publish(debug_im)

        end = time.time()
        print(f'Took {end - start:.2f} secs')


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
