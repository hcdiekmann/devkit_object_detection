
import rclpy  
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class AMRPositioner(Node):
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give node name
        super().__init__('amr_positioner')
        self.obj_marker_sub = self.create_subscription(Marker,'/detection/obj_markers',self.marker_callback, 10)
        self.wander_to_pub = self.create_publisher(PoseStamped,'/wander/to', 10)
        self.pause_wander_pub = self.create_publisher(Bool,'/wander/pause_wandering', 10)
        self.get_logger().info(
                f"AMR positioning node initialised successfully"
            )

    def marker_callback(self, data):
        new_marker = data

        goal_pose = PoseStamped()
        goal_pose.header = new_marker.header
        goal_pose.pose = new_marker.pose
        self.move_to_object_marker(goal_pose)

    def move_to_object_marker(self, pose):
        self.wander_to_pub.publish(pose)

    def pause_wandering(self, pause):
        pause_wandering = Bool()
        pause_wandering.data = True
        self.pause_wander_pub.publish(pause_wandering)



def main(args=None):

    rclpy.init(args=args)

    # Create the node
    amr_positioning_node = AMRPositioner()
    
    # Spin the node so the callback function is called.
    rclpy.spin(amr_positioning_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    amr_positioning_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
        main()