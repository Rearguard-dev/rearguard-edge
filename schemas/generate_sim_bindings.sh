#!/usr/bin/env bash
# Refresh the simulator's committed Python compatibility binding.
# Native C++/Python/TypeScript bindings use schemas/buf.gen.yaml separately.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

"${PYTHON:-python}" -m grpc_tools.protoc \
  -I"$REPO_ROOT/schemas/proto" \
  --python_out="$WORK" \
  "$REPO_ROOT/schemas/proto/harness/v1/enforcement.proto"

GENERATED="$WORK/harness/v1/enforcement_pb2.py"
# Consumers import a flattened module. Refuse new package dependencies instead
# of installing a binding that cannot be imported through that compatibility path.
if grep -q '^from harness' "$GENERATED"; then
  echo "error: generated binding now has package imports; migrate the simulator imports before flattening" >&2
  exit 1
fi
mv "$GENERATED" "$REPO_ROOT/robot/sim/sim_rig/schemas/enforcement_pb2.py"
echo "python -> robot/sim/sim_rig/schemas/enforcement_pb2.py (review and commit)"
