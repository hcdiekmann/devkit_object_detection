
import rclpy  
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import numpy as np
import pyrealsense2 as rs2
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from devkit_inference_interfaces.msg import Objects


class ObjectMarkerPublisher(Node):
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give node name
        super().__init__('object_marker_publisher')
        self.intrinsics = None
        self.cam_info_sub = self.create_subscription(CameraInfo,'/camera/depth/camera_info',self.cam_info_callback, 10)
        self.objects_sub = self.create_subscription(Objects,'/detection/objects', self.detection_callback, 10)
        self.obj_marker_pub = self.create_publisher(Marker,'/detection/obj_markers',10)
        self.get_logger().info(
                f"ObjectMarkerPublisher node initialised successfully"
            )

    def cam_info_callback(self, info):
        if self.intrinsics:
            self.destroy_subscription(self.cam_info_sub)
            return
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = info.width
        self.intrinsics.height = info.height
        self.intrinsics.ppx = info.k[2]
        self.intrinsics.ppy = info.k[5]
        self.intrinsics.fx = info.k[0]
        self.intrinsics.fy = info.k[4]
        if info.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs2.distortion.brown_conrady
        elif info.distortion_model == 'equidistant':
            self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in info.d]
        self.get_logger().info(
                f"Camera depth intrinsics loaded successfully"
            )


    def detection_callback(self, data):
        if self.intrinsics:            
            obj_num = 1
            time_now = self.get_clock().now().to_msg()
            for obj in data.objects:
                if obj.center_pos.z > 0:    # if objects distance is valid create a Marker
                    marker = Marker()
                    marker.header.stamp = time_now
                    marker.header.frame_id = obj.frame_id
                    marker.lifetime = Duration(seconds=0.5).to_msg()
                    marker.type = 2         # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                    marker.id = obj_num
                    
                    marker.scale.x = 0.01
                    marker.scale.y = 0.01
                    marker.scale.z = 0.01
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = obj.confidence

                    # get the 3D point in meters from the pixel coordinate https://dev.intelrealsense.com/docs/projection-in-intel-realsense-sdk-20
                    pointT = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [obj.center_pos.x, obj.center_pos.y ], obj.center_pos.z/1000)
                    marker.pose.position.x = pointT[0]
                    marker.pose.position.y = pointT[1]
                    marker.pose.position.z = pointT[2]
                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0
                    self.obj_marker_pub.publish(marker)
                    obj_num += 1
        else:
            self.get_logger().info(
                f"Waiting for the camera intrinsics to be published"
            )


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    obj_marker_pub_node = ObjectMarkerPublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(obj_marker_pub_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obj_marker_pub_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
        main()


            


