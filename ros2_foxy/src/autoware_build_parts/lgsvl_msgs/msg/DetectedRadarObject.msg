int32 id

geometry_msgs/Vector3 sensor_aim
geometry_msgs/Vector3 sensor_right
geometry_msgs/Point sensor_position
geometry_msgs/Vector3 sensor_velocity
float64 sensor_angle

geometry_msgs/Point object_position
geometry_msgs/Vector3 object_velocity
geometry_msgs/Point object_relative_position
geometry_msgs/Vector3 object_relative_velocity
geometry_msgs/Vector3 object_collider_size
uint8 object_state
bool new_detection

uint8 STATE_MOVING = 0
uint8 STATE_STATIONARY = 1
