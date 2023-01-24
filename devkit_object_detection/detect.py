# subscribes to ROS 2 camera real-time stream and processes it with OpenCV for YOLO object detection  

import time
import rclpy                                                # Python client library for ROS 2
from rclpy.node import Node                                 # Handles the creation of ROS nodes
from sensor_msgs.msg import Image                           # Image is the ROS message type
from std_msgs.msg import Header, String, Bool
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge                              # Package to convert between ROS and OpenCV Images
br = CvBridge()

import numpy as np
import torch
import cv2
print ('OpenCV version: {}' .format(cv2.__version__))


class ObjectDetector(Node):
    """
    Create an ObjectDetector class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give node name
        super().__init__('object_detector')
        self.color_frame = None
        self.depth_frame = None
        self.depth_array = None
        self.detect_interval = 10
        self.frame_count = 0

        # Load and configure YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s6', force_reload=True)
        self.model.iou = 0.45           # NMS IoU threshold
        self.model.conf = 0.50          # NMS confidence threshold
        self.model.multi_label = False  # NMS multiple labels per box
        self.model.classes = None       # (optional list) filter by class, i.e. = [0, 15, 16] for COCO persons, cats and dogs
        self.model.max_det = 100        # maximum number of detections per image

        # Create the camera subscribers, Note: The RealSense D405 model publishes rgb_image to /camera/color/image_rect_raw
        self.color_sub = self.create_subscription(Image,'/camera/color/image_raw', self.color_callback, 10) 
        self.depth_sub = self.create_subscription(Image,'/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        # Create publishers for the inference_image and object markers
        self.inference_pub = self.create_publisher(Image, '/camera/inference_image', 10)


    def color_callback(self, data):
        """
        Callback function for the color frame.
        """
        self.frame_count += 1
        if self.frame_count == self.detect_interval:
            self.frame_count = 0
            ros_image = br.imgmsg_to_cv2(data)
            self.color_frame = cv2.cvtColor(ros_image, cv2.COLOR_BGR2RGB)
            self.depth_array = np.array(self.depth_frame, dtype=np.float64)
            self.detect()
         

    def depth_callback(self, data):
        """
        Callback function for the depth frame.
        """
        self.depth_frame = br.imgmsg_to_cv2(data)
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(ros_depth_frame, alpha=0.08), cv2.COLORMAP_JET)
        #cv2.imshow('Colormap', self.depth_frame)
        #cv2.waitKey(1)


    def detect(self):
        if self.color_frame is not None:
            results = self.model(self.color_frame)
            self.process_predictions(results)
            inference_image = np.squeeze(results.render())      # draw bounding boxes, labels and confidence on source image
            ros_infer_image = br.cv2_to_imgmsg(inference_image) # convert to ROS Image msg
            self.inference_pub.publish(ros_infer_image)         # publish inference image to ros topic
            results.print()                                     # prints inference metrics to terminal
            #cv2.imshow('Inference Image', inference_image )   
            #cv2.waitKey(1)


    def process_predictions(self, predictions):
        objects = predictions.pandas().xyxy[0].to_dict(orient = "records")
        
        if len(predictions) != 0:
            for obj in objects:
                name,confi,x1,y1,x2,y2, = obj['name'],obj['confidence'],obj['xmin'],obj['ymin'],obj['xmax'],obj['ymax']
                
                #calculate bounding box center
                x_center, y_center  = (x1 + x2)/2, (y1 + y2)/2
                object_center = (int(x_center), int(y_center))
                cv2.circle(self.color_frame, object_center, 3, (0,0,255),-1 ) # center point as red circle

                if self.depth_array is not None:
                    # get distance to object center from depth array
                    distance = self.depth_array[object_center[1],object_center[0]]
                    cv2.putText(self.color_frame, "{}mm".format(distance),(object_center[0], object_center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2, cv2.LINE_AA)



def main(args=None):
    rclpy.init(args=args)

    # Create the node
    detection_node = ObjectDetector()

    # Spin the node so the callback function is called.
    rclpy.spin(detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detection_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
        main()
