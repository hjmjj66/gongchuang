std_msgs/Header header

bool connected
bool status_valid
bool odometry_valid
bool pc_control
# Bridge permission, independent of whether a fresh nonzero command exists.
bool motion_allowed

uint8 chassis_status
uint8 arm_state
uint8 online_mask
uint8 error_flags
uint8 odom_status
uint8 control_source

float32 fb_mm
float32 ud_mm
float32 yaw_rad
uint32 board_time_ms
# Current health inhibition, automatically cleared when recovered.
string fault_reason

string odom_session_id
uint64 motion_epoch
bool origin_initialized
bool command_fresh
# Source age of the latest accepted command; -1 when none.
float64 command_age_s
string command_reason
# Last successfully written CDC command (not a claim of physical execution).
uint64 tx_sequence
float64 tx_age_s
float64 sent_vx_mps
float64 sent_vy_mps
float64 sent_wz_rad_s
uint8 sent_mode
