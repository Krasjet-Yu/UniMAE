Header header

# the trajectory id, starts from "1".
uint32 trajectory_id

# the order of trajectory.
uint32 num_order
uint32 num_segment

# the polynomial coecfficients of the trajectory.
float64 start_yaw
float64 final_yaw
float64[] coef_x
float64[] coef_y
float64[] coef_z
float64[] time
float64   mag_coeff
uint32[]  order

#the occ state of trajectory.
uint32 occ_state
string debug_info
