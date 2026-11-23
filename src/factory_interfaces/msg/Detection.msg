# One product measured by the vision node at the pick zone.
std_msgs/Header header
uint32 id
string color            # red | green | blue | yellow | unknown
string shape            # cube | cylinder | unknown
bool defective
string defect_reason    # empty when not defective
float64 x               # world frame, metres (product centroid on the belt)
float64 y
float64 yaw             # orientation on the belt, rad
float64 width           # apparent size, metres
float64 confidence      # 0..1
