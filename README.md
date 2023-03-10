# ROS 2 Object Detection 
A ROS 2 wrapper for inference with [YOLOv5](https://github.com/ultralytics/yolov5#readme) designed for the [UP AMR development kit](https://github.com/AAEONAEU-SW/uprobotic-devkits)

> This package is optimised and tested with Intel® [RealSense™](https://www.intelrealsense.com/) cameras

## Setup Instructions

### Step 1: Install a ROS 2 distribution
 - #### Ubuntu 22.04:
   - [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
 - #### Ubuntu 20.04: 
   - [ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
   - [ROS 2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

### Step 2: Install the Intel® RealSense™ ROS 2 Wrapper
- #### [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/realsense-ros)

___

## **Install**
- #### Clone the packages into your ROS 2 workspace `src`  directory
```bash
cd ~/ros2_ws/src 
git clone https://github.com/hcdiekmann/devkit_object_detection.git
git clone https://github.com/hcdiekmann/devkit_inference_interfaces.git
```
- #### install python dependencies
```bash
pip install -r devkit_object_detection/requirements.txt
```
- #### install ROS 2 dependencies
```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src -r
```
- #### build the package and source the installation
```bash
colcon build
. install/setup.bash
```

## **Run**
- #### Start the RealSense™ camera node
> Set the image size with the launch argument 
eg. 640x480 significantly improves the inference time

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 
```
- #### Start the detection node and marker publisher node
```bash
ros2 run devkit_object_detection obj_detection_node
ros2 run devkit_object_detection marker_pub_node
```
- #### Optionally view the results in RViz
```bash
rviz2
```


## **Example**
### Pretrained detection model
[COCO dataset](https://cocodataset.org/#home) lables for 80 classes of common objects in context

![Result](https://user-images.githubusercontent.com/13176191/215739174-1a26a478-a781-4b26-8ae7-7c00512f1279.png)

## Todo
- [ ] Add launch file with inference and camera parameters
- [ ] Use local inference model or update to YOLOv8


## **Docker**
Build a [custom image](https://www.intel.com/content/www/us/en/develop/documentation/ei4amr-2022-3-developer-guide/top/tutorials-amr/build-docker-images-from-ei-for-amr.html) with the Dockerfile  for usage inside the [Edge Insight for AMR SDK](https://www.intel.com/content/www/us/en/developer/topic-technology/edge-5g/edge-solutions/autonomous-mobile-robots/overview.html) form intel&reg;




