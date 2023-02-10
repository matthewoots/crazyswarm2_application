string id
uint8 IDLE = 0 # Have not taken off
uint8 TAKEOFF = 1 # Taking off sequence
uint8 MOVE = 2 # Move according to external command (change target)
uint8 INTERNAL_TRACKING = 3 # Internal command logic takes precedence over external
uint8 HOVER = 4 # Stop and hover
uint8 LAND = 5 # Landing sequence
uint8 flight_state
bool connected
bool completed
bool mission_capable