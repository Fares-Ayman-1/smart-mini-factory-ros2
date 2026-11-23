# Cell supervisor state, published at 5 Hz.
std_msgs/Header header
string state            # IDLE | RUNNING | PICKING | FAULT | ESTOP
string tower            # green | amber | red | off
string fault            # empty when no fault
uint32 entered          # products counted by IR1
uint32 sorted           # products placed in B1..B4
uint32 rejected         # products placed in B5
uint32[5] bin_counts
float64 items_per_min
float64 reject_rate
