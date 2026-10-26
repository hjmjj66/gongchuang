# Fixed session frame: origin and axes established on entering PC/reconnect.
# Session axes: x initially FORWARD, y initially LEFT; fixed after origin setup.
# Velocities remain BODY-frame x FORWARD, y LEFT, NOT fixed-frame derivatives.
std_msgs/Header header
string odom_session_id
uint64 motion_epoch
uint32 board_time_ms
uint8 odom_status
bool origin_initialized
# Sample/origin health, independent of PC permission. Also check freshness.
bool valid
float64 x_m
float64 y_m
# Continuous, unwrapped chassis yaw, CCW positive.
float64 yaw_rad
float64 yaw_deg
float64 vx_mps
float64 vy_mps
float64 wz_rad_s
