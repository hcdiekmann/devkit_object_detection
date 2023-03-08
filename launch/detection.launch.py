from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
    Node(
            package='devkit_object_detection',
            executable='obj_detection_node'
        ),
    Node(
            package='devkit_object_detection',
            executable='marker_pub_node'
        ),
    Node(
            package='devkit_object_detection',
            executable='amr_positioning_node'
        ),

   ])