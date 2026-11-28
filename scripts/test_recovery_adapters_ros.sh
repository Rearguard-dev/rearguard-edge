#!/usr/bin/env bash
# Build/test only. Does not install, activate a profile or connect to a robot.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
recovery_build_dir="${1:-${repo_root}/edge/build-recovery-ros}"
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is required. Run on a Jazzy development host." >&2
  exit 1
fi
# ROS setup files may refer to unset shell variables.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${REARGUARD_RECOVERY_TEST_DOMAIN_ID:-103}"
export ROS_LOCALHOST_ONLY=1
cmake -S "${repo_root}/edge" -B "${recovery_build_dir}" \
  -DHARNESS_WITH_ROS2=ON -DHARNESS_WITH_RECOVERY_ROS=ON \
  -DHARNESS_WITH_ONNX=OFF -DHARNESS_WITH_TENSORRT=OFF -DHARNESS_BUILD_TESTS=ON
cmake --build "${recovery_build_dir}" --parallel 2 --target \
  harness_recovery_library_tests harness_recovery_adapters_tests harness_recovery_environment_tests \
  harness_recovery_transport_ros_tests harness_ros_library_recovery_tests
ctest --test-dir "${recovery_build_dir}" --output-on-failure \
  -R '^harness_(recovery_(library|adapters|environment|transport_ros)|ros_library_recovery)_tests$'
