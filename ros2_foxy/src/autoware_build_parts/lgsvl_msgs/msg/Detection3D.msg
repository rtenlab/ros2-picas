std_msgs/Header header
uint32 id  # The numeric ID of the detected object
string label  # car, pedestrian
float32 score  # The confidence score of the detected object in the range [0-1]
BoundingBox3D bbox  # A 3D bounding box
geometry_msgs/Twist velocity  # Linear and angular velocity
