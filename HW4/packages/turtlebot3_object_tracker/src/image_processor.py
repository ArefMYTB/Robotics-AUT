#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.engine.results import Results

# ROS
import rospy
from sensor_msgs.msg import Image
from turtlebot3_object_tracker.srv import detection, detectionResponse

class ImageProcessor:
    def __init__(self) -> None:
        # Image message
        self.image_msg = Image()

        self.rec_x = (0,0)
        self.rec_y = (1, 1)

        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        # TODO: Subscribe on your robot's camera topic
        # NOTE: Make sure you use the provided listener for this subscription
        self.camera_subscriber = rospy.Subscriber('/follower/camera/image', Image, self.camera_listener)


        # TODO: Instantiate your YOLO object detector/classifier model
        self.model: YOLO = YOLO()
        # TODO: You need to update results each time you call your model
        self.results: Results = None

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        # TODO: Setup your "human detection" service
        self.human_detection_server = rospy.Service('srv_detection', detection, self.srv_detection)


        self.view()


    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)

    def srv_detection(self, request):
        self.results = self.model(self.image_np)

        self.label = request.object_label

        response = detectionResponse()
        response.object_info = self.detect(self.results)
        response.image_size = self.image_res[:2]  # Send only height and width
        return response
    
    def detect(self, results):
        
        object_info = []

        for detection in results:
            for i in range(detection.boxes.shape[0]):
                detected_label = detection.names[int(detection.boxes.cls[i].item())]
                confidence = float(detection.boxes.conf[i].item())
                
                if detected_label == self.label and confidence > 0.5:
                    bounding_box = detection.boxes.xyxy[i].cpu().numpy()
                    self.rec_x = (int(bounding_box[0]), int(bounding_box[1]))
                    self.rec_y = (int(bounding_box[2]), int(bounding_box[3]))
                    object_info.append(int(bounding_box[0])) 
                    object_info.append(int(bounding_box[2])) 
                    return object_info
                else:
                    self.rec_x = (0,0)
                    self.rec_y = (1, 1)
        
        return object_info

    def view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0: # If there is no image data
                    continue

                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                # frame = copy.deepcopy(self.image_np)
                frame = self.image_np.copy()

                cv2.rectangle(frame, self.rec_x, self.rec_y, (0, 255, 0), 2)
                cv2.imshow("robot_view", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass


if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()


