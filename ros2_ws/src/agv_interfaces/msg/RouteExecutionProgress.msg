# Optional debug snapshot from the executor's actual merged motion segment.
std_msgs/Header header
string run_id
string request_id
string executor_session_id
string state
bool feedback_valid
uint32 segment_index
uint8 opcode
int32 from_node
int32 to_node
string start_heading
float64 amount
float64 progress
float64 baseline_yaw_deg
float64 yaw_deg

string odom_session_id
uint64 motion_epoch
float64 baseline_x_m
float64 baseline_y_m
float64 x_m
float64 y_m
float64 direction_x
float64 direction_y
# Signed translation projection and cross-track offset; NOT cumulative distance.
float64 signed_progress_mm
float64 cross_track_mm
