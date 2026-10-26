cmake_minimum_required(VERSION 3.8)
project(agv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Humble's adapter reads its generated path list as ASCII by default, truncating
# Chinese workspace paths. Apply the UTF-8 fix locally, without editing /opt/ros.
set(_adapter_module "${rosidl_adapter_DIR}/rosidl_adapt_interfaces.cmake")
file(READ "${_adapter_module}" _adapter_source)
string(REPLACE
  "file(STRINGS \"\${idl_output}\" idl_tuples)"
  "file(STRINGS \"\${idl_output}\" idl_tuples ENCODING UTF-8)"
  _adapter_utf8 "${_adapter_source}")
if(NOT "${_adapter_utf8}" STREQUAL "${_adapter_source}")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapt_interfaces_utf8.cmake"
    "${_adapter_utf8}")
  include("${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapt_interfaces_utf8.cmake")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Edge.msg"
  "msg/BlockedEdges.msg"
  "msg/OdometryFeedback.msg"
  "msg/CBoardStatus.msg"
  "msg/CBoardOdometry.msg"
  "msg/ControlCommand.msg"
  "msg/RouteControl.msg"
  "msg/RouteExecutionStatus.msg"
  "msg/RouteExecutionProgress.msg"
  "msg/RouteStep.msg"
  "msg/RoutePlan.msg"
  "srv/PlanRoute.srv"
  "msg/ExecutorRuntime.msg"
  "msg/FirstLayerRecord.msg"
  "msg/WorkStepRecord.msg"
  "msg/MissionState.msg"
  "msg/MissionTaskInfo.msg"
  "srv/GetExecutorRuntime.srv"
  "srv/StopExecutor.srv"
  "srv/ResetExecutor.srv"
  "action/ScanRoads.action"
  "action/RunWorkBatch.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
