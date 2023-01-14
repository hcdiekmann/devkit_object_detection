# ROS 2 Object Detection 
A ROS 2 wrapper for inference with [YOLOv5](https://github.com/ultralytics/yolov5#readme) (and soon [YOLOv8](https://github.com/ultralytics/ultralytics#readme)) designed for the [UP AMR development kit](https://github.com/AAEONAEU-SW/uprobotic-devkits)

> This package is optimised and tested with the intel [Realsense D435i](https://www.intelrealsense.com/depth-camera-d435i/) camera

### Pretrained detection model
[COCO dataset](https://cocodataset.org/#home) lables for 80 classes of common objects in context

## Install

```bash
cd /ros2_ws/src 
git clone https://github.com/hcdiekmann/devkit_object_detection.git
pip install -r requirements.txt
cd ros2_ws
colcon build 
```

## Run



``` bash
ros2 run devkit_object_detection detection_node
```


## Build with Docker
Build your custom image for usage inside the [EI for AMR SDK](https://www.intel.com/content/www/us/en/developer/topic-technology/edge-5g/edge-solutions/autonomous-mobile-robots/overview.html) form intel&reg;




