# 路径节点返回给mission的控制反馈；C板通信只订阅mission输出的ControlCommand。
std_msgs/Header header
string run_id
string executor_session_id
string source
string request_id
# 同一任务内从1开始递增，用于丢弃重复或乱序反馈。
uint64 sequence
agv_interfaces/ControlCommand command
# EXECUTING / COMPLETED / ABORTED；终态command必须全零。
string state
string reason
