# This message is used specifically in the devkit_object_detection package
#
# Message that holds information about each detected object

Header header                       # standard header
string frame_id                     # parent frame of the object (usually 'camera_depth_optical_frame' or similar)

string lable                        # class lable of the object eg. person or car
float32 confidence                  # prediction confidence as a percentage

float32 x1                          # top left x coordinate of bbox
float32 y1                          # top left y coordinate of bbox
float32 x2                          # bottom right x coordinate of bbox
float32 y2                          # bottom right y coordinate of bbox

geometry_msgs/Point center_pos      # x, y, z position of the objects bbox center
