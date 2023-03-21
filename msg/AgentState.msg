string id
uint8 IDLE = 0 # Have not taken off
uint8 TAKEOFF = 1 # Taking off sequence
uint8 MOVE = 2 # Move according to high level command (change target)
uint8 MOVE_VELOCITY = 3 # Move according to velocity command (change target)
uint8 INTERNAL_TRACKING = 4 # Internal command logic takes precedence over external
uint8 HOVER = 5 # Stop and hover
uint8 LAND = 6 # Landing sequence
uint8 EMERGENCY = 7 # Emergency
uint8 flight_state
bool connected
bool completed
bool mission_capable