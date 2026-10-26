uint8 FORWARD_ONE_EDGE=1
uint8 TURN_LEFT_90=2
uint8 TURN_RIGHT_90=3
uint8 STRAFE_LEFT_ONE_EDGE=4
uint8 STRAFE_RIGHT_ONE_EDGE=5
uint8 BACKWARD_ONE_EDGE=6

# Debug-only discrete action description. Execution publishes velocity instead.
uint32 step_index
uint8 opcode
int32 from_node
int32 to_node
# Translation: configured length of this edge. Rotation: zero.
uint32 distance_mm
# Planned heading after this step, not actual execution feedback.
string target_heading
