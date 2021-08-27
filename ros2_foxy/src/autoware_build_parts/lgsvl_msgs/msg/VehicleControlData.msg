std_msgs/Header header

float32 acceleration_pct  # 0 to 1
float32 braking_pct  # 0 to 1
float32 target_wheel_angle  # radians
float32 target_wheel_angular_rate  # radians / second
uint8 target_gear

uint8 GEAR_NEUTRAL = 0
uint8 GEAR_DRIVE = 1
uint8 GEAR_REVERSE = 2
uint8 GEAR_PARKING = 3
uint8 GEAR_LOW = 4
