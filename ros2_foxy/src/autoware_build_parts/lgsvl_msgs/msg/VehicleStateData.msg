std_msgs/Header header

uint8 blinker_state
uint8 headlight_state
uint8 wiper_state
uint8 current_gear
uint8 vehicle_mode
bool hand_brake_active
bool horn_active
bool autonomous_mode_active

uint8 BLINKERS_OFF = 0
uint8 BLINKERS_LEFT = 1
uint8 BLINKERS_RIGHT = 2
uint8 BLINKERS_HAZARD = 3

uint8 HEADLIGHTS_OFF = 0
uint8 HEADLIGHTS_LOW = 1
uint8 HEADLIGHTS_HIGH = 2

uint8 WIPERS_OFF = 0
uint8 WIPERS_LOW = 1
uint8 WIPERS_MED = 2
uint8 WIPERS_HIGH = 3

uint8 GEAR_NEUTRAL = 0
uint8 GEAR_DRIVE = 1
uint8 GEAR_REVERSE = 2
uint8 GEAR_PARKING = 3
uint8 GEAR_LOW = 4

uint8 VEHICLE_MODE_COMPLETE_MANUAL = 0
uint8 VEHICLE_MODE_COMPLETE_AUTO_DRIVE = 1
uint8 VEHICLE_MODE_AUTO_STEER_ONLY = 2
uint8 VEHICLE_MODE_AUTO_SPEED_ONLY = 3
uint8 VEHICLE_MODE_EMERGENCY_MODE = 4
