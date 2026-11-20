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

# v0.7: signed pulses relative to the homing zero (3200 pulses/revolution).
int32 fb_pulses
int32 ud_pulses
# bit0: FB valid; bit1: UD valid; bit2: zero calibration completed.
uint8 pos_flags
# Gantry single-turn absolute angle; chassis yaw remains continuous.
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
