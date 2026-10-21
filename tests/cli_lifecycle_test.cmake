if(NOT DEFINED CLI)
  message(FATAL_ERROR "CLI is required")
endif()

set(TEST_ROOT "${CMAKE_CURRENT_BINARY_DIR}/cli-lifecycle-test")
set(STATE_ROOT "${TEST_ROOT}/rearguard")
file(REMOVE_RECURSE "${TEST_ROOT}")
file(MAKE_DIRECTORY "${STATE_ROOT}/service/sessions/session-1")
file(WRITE "${STATE_ROOT}/events.jsonl" "{}\n")
file(WRITE "${STATE_ROOT}/service/sessions/session-1/recording.mcap" "evidence")

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env "REARGUARD_STATE_DIR=${STATE_ROOT}"
          "${CLI}" help
  RESULT_VARIABLE HELP_RESULT
  OUTPUT_VARIABLE HELP_OUTPUT
  ERROR_VARIABLE HELP_ERROR)
if(NOT HELP_RESULT EQUAL 0)
  message(FATAL_ERROR "help command failed: ${HELP_ERROR}${HELP_OUTPUT}")
endif()
foreach(EXPECTED
    "rearguard start"
    "rearguard stop"
    "rearguard purge"
    "rearguard connect"
    "rearguard events"
    "rearguard validate"
    "rearguard show"
    "rearguard replay"
    "rearguard verify"
    "rearguard hash"
    "rearguard diff"
    "rearguard abi"
    "rearguard scan setup"
    "rearguard scan runtime"
    "rearguard observe start"
    "rearguard observe candidate"
    "rearguard observe activate"
    "rearguard workflow init"
    "rearguard workflow apply"
    "rearguard workflow sync-once"
    "rearguard service uploader"
    "rearguard uninstall")
  string(FIND "${HELP_OUTPUT}" "${EXPECTED}" FOUND_AT)
  if(FOUND_AT EQUAL -1)
    message(FATAL_ERROR "help output omitted ${EXPECTED}")
  endif()
endforeach()
if(EXISTS "${STATE_ROOT}/events.jsonl")
  file(READ "${STATE_ROOT}/events.jsonl" EVENTS_AFTER_HELP)
  if(NOT EVENTS_AFTER_HELP STREQUAL "{}\n")
    message(FATAL_ERROR "help command mutated the event journal")
  endif()
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env "REARGUARD_STATE_DIR=${STATE_ROOT}"
          "${CLI}" start --help
  RESULT_VARIABLE START_HELP_RESULT
  OUTPUT_VARIABLE START_HELP_OUTPUT
  ERROR_VARIABLE START_HELP_ERROR)
if(NOT START_HELP_RESULT EQUAL 0 OR
   NOT START_HELP_OUTPUT MATCHES "rearguard service start")
  message(FATAL_ERROR "start alias help failed: ${START_HELP_ERROR}${START_HELP_OUTPUT}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env "REARGUARD_STATE_DIR=${STATE_ROOT}"
          "${CLI}" stop
  RESULT_VARIABLE STOP_RESULT
  OUTPUT_VARIABLE STOP_OUTPUT
  ERROR_VARIABLE STOP_ERROR)
if(NOT STOP_RESULT EQUAL 0 OR NOT STOP_OUTPUT MATCHES "not_running")
  message(FATAL_ERROR "stop alias failed: ${STOP_ERROR}${STOP_OUTPUT}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env "REARGUARD_STATE_DIR=${STATE_ROOT}"
          "${CLI}" purge --dry-run --json
  RESULT_VARIABLE DRY_RUN_RESULT
  OUTPUT_VARIABLE DRY_RUN_OUTPUT
  ERROR_VARIABLE DRY_RUN_ERROR)
if(NOT DRY_RUN_RESULT EQUAL 0 OR NOT DRY_RUN_OUTPUT MATCHES "\"dry_run\":true")
  message(FATAL_ERROR "purge dry run failed: ${DRY_RUN_ERROR}${DRY_RUN_OUTPUT}")
endif()
if(NOT EXISTS "${STATE_ROOT}/service/sessions/session-1/recording.mcap")
  message(FATAL_ERROR "purge dry run removed recording data")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env "REARGUARD_STATE_DIR=${STATE_ROOT}"
          "${CLI}" purge --yes --json
  RESULT_VARIABLE PURGE_RESULT
  OUTPUT_VARIABLE PURGE_OUTPUT
  ERROR_VARIABLE PURGE_ERROR)
if(NOT PURGE_RESULT EQUAL 0 OR NOT PURGE_OUTPUT MATCHES "\"removed\":true")
  message(FATAL_ERROR "purge failed: ${PURGE_ERROR}${PURGE_OUTPUT}")
endif()
if(EXISTS "${STATE_ROOT}")
  message(FATAL_ERROR "purge left or recreated the state root")
endif()

file(REMOVE_RECURSE "${TEST_ROOT}")
