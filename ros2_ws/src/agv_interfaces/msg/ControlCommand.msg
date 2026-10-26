# Body-frame target velocity: x FORWARD, y LEFT; CCW positive.
float64 vx_mps
float64 vy_mps
float64 omega_deg_s
# Automatic tasks bind to the current odometry/permission generation.
# CLI commands may leave these at their defaults. Zero is always a stop.
string odom_session_id
uint64 motion_epoch
