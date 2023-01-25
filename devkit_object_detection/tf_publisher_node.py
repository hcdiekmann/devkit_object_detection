import rclpy  
from rclpy.node import Node
import tf2_ros
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from msg import Objects


class ObjectTransformPublisher(Node):
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give node name
        super().__init__('object_tf_publisher')
        
        self.objects_sub = self.create_subscribtion(Objects,'/detection/objects', self.detection_callback, 10)
        self.tfb = tf2_ros.TransformBroadcaster()


    def detection_callback(self, data):
        objects = data
        obj_num = 1
        final_tfs = []
        time_now = rclpy.time.now()
        for obj in objects:
            tf = TransformStamped()
            tf.header.frame_id = obj.frame_id
            tf.header.stamp = time_now
            tf.child_frame_id = '{obj.lable}_' + str(obj_num)
            tf.transform.translation.x = obj.center_pos.x
            tf.transform.translation.y = obj.center_pos.y
            tf.transform.translation.z = obj.center_pos.z
            tf.transform.rotation = Quaternion(0,0,0,1)
            obj_num += 1
            final_tfs.append(tf)

        self.tfb.sendTransform(final_tfs)



def main(args=None):
    rclpy.init(args=args)

    # Create the node
    obj_tf_pub_node = ObjectTransformPublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(obj_tf_pub_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obj_tf_pub_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
        main()


            


