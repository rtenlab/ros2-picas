std_msgs/Header header

float32 speed_mps
float32 throttle_pct  # 0 to 1
float32 brake_pct     # 0 to 1
float32 steer_pct     # -1 to 1
bool parking_brake_active
bool high_beams_active
bool low_beams_active
bool hazard_lights_active
bool fog_lights_active
bool left_turn_signal_active
bool right_turn_signal_active
bool wipers_active
bool reverse_gear_active
int8 selected_gear
bool engine_active
float32 engine_rpm
float64 gps_latitude
float64 gps_longitude
float64 gps_altitude
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 linear_velocities

int8 GEAR_NEUTRAL = 0
int8 GEAR_DRIVE = 1
int8 GEAR_REVERSE = 2
int8 GEAR_PARKING = 3
int8 GEAR_LOW = 4
