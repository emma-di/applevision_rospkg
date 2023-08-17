#!/usr/bin/python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
from applevision_rospkg.msg import RegionOfInterestWithConfidenceStamped
from helpers import HeaderCalc
from applevision_vision import MODEL_PATH
from ultralytics import YOLO
import time
import numpy as np

CONFIDENCE_THRESH = 0.7
CLASESS_YOLO = ['Green Apple', 'Red Apple'] # we don't actually use green apple, I just didn't want to retrain

def format_yolov8(
    source
):  #Function taken from medium: https://medium.com/mlearning-ai/detecting-objects-with-yolov5-opencv-python-and-c-c7cf13d1483c
    # YOLO needs a square image; expand the image so it is square
    col, row, _ = source.shape
    _max = max(col, row)
    resized = np.zeros((_max, _max, 3), np.uint8)
    resized[0:col, 0:row] = source
    result=cv2.resize(source, (640,640))
    return result

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
        model = self.net
        
        if rospy.Time.now() - im.header.stamp > rospy.Duration.from_sec(0.1):
            # this frame is too old
            rospy.logwarn_throttle_identical(3, 'CV: Ignoring old frame')
            return None
        start = time.time()
        
        frame = self._br.imgmsg_to_cv2(im, 'bgr8')
        adjusted_image = format_yolov8(frame)
            
        results = model.predict(adjusted_image, stream = False, classes = [1], conf=CONFIDENCE_THRESH, imgsz=(640,640))
        rows = results[0].boxes.shape[0]
        
        class_ids, confs, boxes = list(), list(), list()
        image_height, image_width, _ = frame.shape
        x_factor = image_width/640
        y_factor = image_height/640
        
        for i in range(rows):
            row = results[0].boxes.data[i]
            conf=row[4]
            class_id=row[5]
            
            # check if confidence is high enough; if yes, record the box
            if conf>CONFIDENCE_THRESH:
                label = CLASESS_YOLO[int(class_id)] # only does red apples bc in line 57 we specify to only predict class 1
                class_ids.append(label)
                confs.append(conf)
                
                # extract boxes (w and h are actually x2 and y2, but I didn't change it to not confuse the nodes)
                x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item()
                left = int(x*x_factor) # x1
                top = int(y*y_factor) # y1
                right = int(w*x_factor) # x2
                bottom=int(h*y_factor)# y2
                box = np.array([left, top, right, bottom])
                boxes.append(box)
        
        # TODO: they used indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.25, 0.45) in original code to suppress weak/overlapping boxes? how do you choose to just latch onto one apple?
        if confs:
            # only stores and draws the max confidence box        
            max_id = confs.index(max(confs))
            max_box = boxes[max_id]
            cv2.rectangle(frame, (max_box[0], max_box[1]), (max_box[2], max_box[3]), (0,255,0), 3)  
            cv2.putText(frame, f"Apple: {confs[max_id]:.2}", (max_box[0] + 5, max_box[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1)
            # publishing the position to /applevision/est_apple_pos
            msg = RegionOfInterestWithConfidenceStamped(
                header=self._header.get_header(),
                x=max_box[0],
                y=max_box[1],
                w=(max_box[2]-x),
                h=(max_box[3]-y),
                image_w=640,
                image_h=360,
                confidence = confs[max_id])
            self.pub.publish(msg) # publishes to palm_camera
        
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
            self.pub.publish(msg) # publishes 0 to palm_camera if no objects detected
            #TODO: this could be why it still moves when no objects detected?
        
        # sending it to RViz
        debug_im = self._br.cv2_to_imgmsg(frame)
        self.pub_debug.publish(debug_im)
        
        end = time.time()
        print(f'Took {end - start:.2f} secs')
 
if __name__ == '__main__':
    # TODO: WHY?
    rospy.init_node('applevision_fake_sensor_data')
    
    # load YOLOv8 model
    net = YOLO('/root/catkin_ws/src/applevision_rospkg/src/applevision_vision/best-roboflow.pt')

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