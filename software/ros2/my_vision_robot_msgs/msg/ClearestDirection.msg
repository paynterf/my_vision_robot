# ClearestDirection.msg
std_msgs/Header header

# Best sector (0 = 9:00 ... 6 = 3:00)
int8 best_sector

# Recommended turn in degrees
# Positive = CW (right), Negative = CCW (left), 0 = straight
float32 turn_deg

# Average depth of the best sector (mm). 0 = no valid data
float32 best_depth_mm

# Simple status
# 0 = OK
# 1 = TOO_CLOSE
# 2 = NO_VALID_DATA
uint8 status