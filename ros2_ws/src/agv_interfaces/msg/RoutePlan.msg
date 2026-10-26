string run_id
string request_id
# 0 means standalone debugging; 1..15 are mission stages.
uint8 stage
int32 start
int32 goal
# ok / no_path / invalid_request / map_not_ready / odometry_not_ready /
# odometry_invalid / execution_in_progress / duplicate_request_id
string status
string reason
# On any failure, both arrays are empty. On success nodes includes both endpoints.
int32[] nodes
agv_interfaces/RouteStep[] actions
uint64 map_revision
uint64 total_distance_mm
# Changes of translation direction; excludes terminal work rotations.
uint32 direction_changes
string start_heading
# Planned final heading; empty on failure. This is debug output, not feedback.
string end_heading
