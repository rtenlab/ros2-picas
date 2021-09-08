std_msgs/Header header
uint32 id  # The numeric ID of the detected signal
string label  # green, yellow, red
float32 score  # The confidence score of the detected signal in the range [0-1]
BoundingBox3D bbox  # A 3D bounding box
