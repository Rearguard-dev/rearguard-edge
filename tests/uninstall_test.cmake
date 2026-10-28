if(NOT DEFINED CLI)
  message(FATAL_ERROR "CLI is required")
endif()

set(TEST_ROOT "${CMAKE_CURRENT_BINARY_DIR}/uninstall-test")
set(HOME_ROOT "${TEST_ROOT}/home")
set(PREFIX "${HOME_ROOT}/.harness")
set(STATE_ROOT "${HOME_ROOT}/.local/state/rearguard")
set(CONFIG_HOME "${HOME_ROOT}/.config")
set(CONFIG_ROOT "${CONFIG_HOME}/rearguard")
file(REMOVE_RECURSE "${TEST_ROOT}")
file(MAKE_DIRECTORY
  "${PREFIX}/bin" "${PREFIX}/lib" "${PREFIX}/include" "${PREFIX}/share"
  "${STATE_ROOT}/service/sessions" "${CONFIG_ROOT}")
file(COPY "${CLI}" DESTINATION "${PREFIX}/bin")
get_filename_component(CLI_NAME "${CLI}" NAME)
if(NOT CLI_NAME STREQUAL "rearguard")
  file(RENAME "${PREFIX}/bin/${CLI_NAME}" "${PREFIX}/bin/rearguard")
endif()
file(WRITE "${PREFIX}/lib/libharness_kernel.so" "install marker")
file(WRITE "${PREFIX}/include/installed.h" "install marker")
file(WRITE "${PREFIX}/share/installed.json" "{}\n")
file(WRITE "${STATE_ROOT}/events.jsonl" "{}\n")
file(WRITE "${STATE_ROOT}/service/sessions/recording.mcap" "evidence")
file(WRITE "${CONFIG_ROOT}/device.json" "{}\n")
file(WRITE "${HOME_ROOT}/.profile"
  "keep=this\n\n# added by harness-kernel install.sh\nexport PATH=\"${PREFIX}/bin:$PATH\"\n")

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env
          "HOME=${HOME_ROOT}"
          "XDG_CONFIG_HOME=${CONFIG_HOME}"
          "REARGUARD_STATE_DIR=${STATE_ROOT}"
          "${PREFIX}/bin/rearguard" uninstall --prefix "${PREFIX}" --yes --json
  RESULT_VARIABLE RESULT
  OUTPUT_VARIABLE OUTPUT
  ERROR_VARIABLE ERROR)
if(NOT RESULT EQUAL 0 OR NOT OUTPUT MATCHES "\"removed\":true")
  message(FATAL_ERROR "uninstall failed: ${ERROR}${OUTPUT}")
endif()
foreach(REMOVED
    "${PREFIX}/bin" "${PREFIX}/lib" "${PREFIX}/include" "${PREFIX}/share"
    "${STATE_ROOT}" "${CONFIG_ROOT}")
  if(EXISTS "${REMOVED}")
    message(FATAL_ERROR "uninstall left ${REMOVED}")
  endif()
endforeach()
file(READ "${HOME_ROOT}/.profile" PROFILE)
if(PROFILE MATCHES "harness-kernel install.sh" OR PROFILE MATCHES "${PREFIX}/bin")
  message(FATAL_ERROR "uninstall left its shell PATH block")
endif()
if(NOT PROFILE MATCHES "keep=this")
  message(FATAL_ERROR "uninstall modified unrelated shell configuration")
endif()

file(REMOVE_RECURSE "${TEST_ROOT}")
