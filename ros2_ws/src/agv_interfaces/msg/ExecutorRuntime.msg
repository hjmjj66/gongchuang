# Identity changes on process restart. All readiness flags describe this executor.
string run_id
string executor_session_id
string source
bool busy
bool quiescent
bool motion_released
bool arm_pending
bool travel_ready
bool config_ready
string active_task_id
string last_task_id
string last_state
string reason
int32 home_node
int32 current_node
bool position_known
string current_heading
bool heading_known
float64 yaw_deg
bool odometry_ready
bool map_ready
uint64 map_revision
agv_interfaces/Edge[] blocked_edges

string odom_session_id
uint64 motion_epoch
