uint8 NONE=0
uint8 TAKEOFF=1
uint8 LAND=2
uint8 HOVER=3
uint8 ABORT=4
uint8 RESET=5

std_msgs/Header header
uint8 command
float32 takeoff_height
uint32 request_id
