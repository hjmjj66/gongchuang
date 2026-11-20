# Continuous raw-area visual servo. A new stream_id is required after each stop/action.
std_msgs/Header header
string stream_id
bool enabled
float64 yaw_wz
float64 fb_rpm
string odom_session_id
uint64 motion_epoch
