#include "help.hpp"

#include <cstdio>

namespace harness::cli {

void print_help() {
    std::fputs(
        R"HELP(Rearguard edge CLI

Usage:
  rearguard <command> [options]

Help and version:
  rearguard help
      Example: rearguard help
  rearguard --version
      Example: rearguard --version

Service lifecycle:
  rearguard start [--config FILE] [--robot-id UUID] [--domain-id N]
                  [--storage mcap|sqlite3] [--credentials FILE] [--backend-url URL]
      Example: rearguard start
  rearguard stop
      Example: rearguard stop
  rearguard purge [--dry-run] [--yes] [--json]
      Example: rearguard purge --dry-run
  rearguard service start [service options]
      Example: rearguard service start --domain-id 71 --storage mcap
  rearguard service status
      Example: rearguard service status
  rearguard service stop
      Example: rearguard service stop
  rearguard service uploader --store DIR (--session DIR|--sessions-root DIR)
                             [--credentials FILE] [--backend-url URL]
                             [--interval-ms 2000]
      Example: rearguard service uploader --store ./profiles/robot-1 \
               --sessions-root ./sessions

Pairing and events:
  rearguard connect [--name DEVICE] [--scan-settle-ms 5000]
                    [--scan-parallelism 16] [--json]
      Example: rearguard connect --name warehouse-arm
  rearguard events [--after N] [--follow] [--json]
      Example: rearguard events --after 100 --follow

Profiles and recorded evidence:
  rearguard validate PROFILE [--json]
      Example: rearguard validate ./profile.yaml
  rearguard show PROFILE [--json]
      Example: rearguard show ./profile.yaml --json
  rearguard replay --profile PROFILE --trace TRACE [--out EPISODE]
                   [--critic-lib-dir DIR] [--json]
      Example: rearguard replay --profile ./profile.yaml --trace ./trace.jsonl \
               --out ./episode.mcap
  rearguard verify EPISODE [--expect-hash HEX] [--json]
      Example: rearguard verify ./episode.mcap
  rearguard hash PROFILE [--json]
      Example: rearguard hash ./profile.yaml
  rearguard hash --joints J0,J1,J2 [--json]
      Example: rearguard hash --joints shoulder,elbow,wrist
  rearguard diff EPISODE_A EPISODE_B [--json]
      Example: rearguard diff ./expected.mcap ./actual.mcap
  rearguard abi
      Example: rearguard abi

ROS discovery and observation:
  rearguard scan setup [--domain-min 0] [--domain-max 232] [--settle-ms 750]
                       [--parallelism 16] [--out INVENTORY]
      Example: rearguard scan setup --out ./inventory.json
  rearguard scan runtime --config FILE --session DIR [--store DIR]
                         [--storage mcap|sqlite3] --domain-id N
                         [--wall-timeout 1800]
      Example: rearguard scan runtime --config ./binding.json \
               --session ./session --domain-id 71
  rearguard observe start --config FILE --session DIR [--store DIR]
                          [--storage mcap|sqlite3] [--domain-id N]
                          [--wall-timeout 1800]
      Example: rearguard observe start --config ./binding.json \
               --session ./session --domain-id 71
  rearguard observe profile (--config FILE|--session DIR)
      Example: rearguard observe profile --session ./session
  rearguard observe status --session DIR
      Example: rearguard observe status --session ./session
  rearguard observe capture --session DIR --label TEXT [--before 5] [--after 3]
      Example: rearguard observe capture --session ./session --label "grasp failed"
  rearguard observe candidate --session DIR --evidence-class TEXT
                              --confidence NUMBER [--detector-id TEXT]
                              [--before 5] [--after 3] [--label TEXT]
      Example: rearguard observe candidate --session ./session \
               --evidence-class collision --confidence 0.98
  rearguard observe windows --session DIR
      Example: rearguard observe windows --session ./session
  rearguard observe proposals --session DIR
      Example: rearguard observe proposals --session ./session
  rearguard observe promote --session DIR --window ID --description TEXT
                            --operator TEXT
      Example: rearguard observe promote --session ./session --window win-1 \
               --description "unexpected contact" --operator alex
  rearguard observe activate --session DIR
      Example: rearguard observe activate --session ./session

Persistent workflow:
  rearguard workflow init --store DIR --config FILE [--profile-id ID]
      Example: rearguard workflow init --store ./profiles/robot-1 \
               --config ./binding.json
  rearguard workflow show --store DIR
      Example: rearguard workflow show --store ./profiles/robot-1
  rearguard workflow outbox --store DIR [--after N]
      Example: rearguard workflow outbox --store ./profiles/robot-1 --after 20
  rearguard workflow artifact --store DIR --kind KIND --file FILE
      Example: rearguard workflow artifact --store ./profiles/robot-1 \
               --kind demonstration_mcap --file ./demo.mcap
  rearguard workflow apply --store DIR --operation OP --payload FILE
                           --actor TEXT --expected-revision N [--request-id ID]
      Example: rearguard workflow apply --store ./profiles/robot-1 \
               --operation accept_failure --payload ./failure.json \
               --actor operator --expected-revision 4
  rearguard workflow export-window --store DIR --session DIR --candidate ID
      Example: rearguard workflow export-window --store ./profiles/robot-1 \
               --session ./session --candidate candidate-1
  rearguard workflow generation-request --store DIR --job ID --out DIR
      Example: rearguard workflow generation-request --store ./profiles/robot-1 \
               --job job-1 --out ./generation-request
  rearguard workflow sync --store DIR --robot-id UUID [--credentials FILE]
                          [--backend-url URL] [--interval-ms 2000]
      Example: rearguard workflow sync --store ./profiles/robot-1 \
               --robot-id 00000000-0000-0000-0000-000000000000
  rearguard workflow sync-once --store DIR --robot-id UUID [--credentials FILE]
                               [--backend-url URL] [--interval-ms 2000]
      Example: rearguard workflow sync-once --store ./profiles/robot-1 \
               --robot-id 00000000-0000-0000-0000-000000000000

Installation:
  rearguard uninstall [--prefix DIR] [--dry-run] [--yes] [--json]
      Example: rearguard uninstall --dry-run

Use `rearguard <group> <subcommand> --help` for concise grouped syntax.
)HELP",
        stdout);
}

}  // namespace harness::cli
