foreach(file
    "${ROOT}/observation/native/ros.cpp"
    "${ROOT}/observation/native/ros_recovery.cpp"
    "${ROOT}/ros2/harness_controller/bringup/config/harness_controllers.yaml"
    "${ROOT}/ros2/harness_controller/bringup/launch/harness_mock.launch.py")
  file(READ "${file}" content)
  string(APPEND all_content "\n${content}")
endforeach()

foreach(required
    "/rearguard/observation/"
    "/rearguard/diagnostics"
    "/rearguard/robot_description"
    "rearguard_observer_"
    "rearguard_setup_scan_"
    "rearguard_observation_cli_"
    "rearguard_controller_manager"
    "rearguard_harness_controller"
    "rearguard_upstream_effort_controller"
    "rearguard_joint_state_broadcaster"
    "must start with rearguard_")
  string(FIND "${all_content}" "${required}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "missing required Rearguard ROS topology prefix: ${required}")
  endif()
endforeach()

foreach(forbidden
    "/harness/observation/"
    "\ncontroller_manager:"
    "\nharness_controller:"
    "\nupstream_effort_controller:")
  string(FIND "${all_content}" "${forbidden}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR "unprefixed Rearguard ROS topology entity remains: ${forbidden}")
  endif()
endforeach()
