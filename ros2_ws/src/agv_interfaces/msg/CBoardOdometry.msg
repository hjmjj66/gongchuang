# Diagnostic copy of C-board ODOM, independent of PC motion permission.
# header.stamp approximates local receive time; it is not board-clock sync.
std_msgs/Header header
uint32 board_time_ms
# Board status bits: bit0=valid, bit1=slip, bit2=IMU OK, bit3=encoders OK.
uint8 odom_status
# Sample health only; not permission to move. Check timestamp for freshness.
bool valid
# Board startup odom frame: x FORWARD, y LEFT, position in meters.
float64 x_m
float64 y_m
# Continuous chassis yaw, CCW positive; both units refer to the same angle.
float64 yaw_rad
float64 yaw_deg
# Board BODY frame velocities: vx FORWARD, vy LEFT, in m/s.
# Same body axes as ControlCommand. Positions/yaw remain RAW board values.
float64 vx_mps
float64 vy_mps
float64 wz_rad_s
