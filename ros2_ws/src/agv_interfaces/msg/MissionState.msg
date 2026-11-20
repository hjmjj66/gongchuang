string run_id
string mode
string state
uint8 stage
string stage_name
string substate
string active_task_id
string active_task_kind
uint8 batch
uint32 confirmed_step_count
# Completed controller actions in this round; not sensor-verified success.
uint32 pick_count
# Includes PLACE and STACK.
uint32 place_count
bool quiescent
bool arm_pending
int32 confirmed_node
bool position_known
string current_heading
uint64 map_revision
string reason
float64 elapsed_s
float64 stage_elapsed_s
