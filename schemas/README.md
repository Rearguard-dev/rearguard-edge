# Schemas — single source of truth

`proto/harness/v1/` defines the harness profile, critic declarations, enforcement
records and related contracts. Consumers generate bindings from these files;
no consumer edits its own wire definitions.

## Native and application bindings

Generate C++, Python and TypeScript bindings against the local protobuf
compiler using `buf.gen.yaml`:

```sh
cd schemas
npm ci
buf generate
```

This writes `schemas/gen/cpp`, `schemas/gen/python` and `schemas/gen/ts`.
These outputs are ignored by Git and generated on each build machine. Use the
same system protobuf compiler/runtime as the native build and ROS dependencies.
CI and the standalone edge release run this generation before building.

From the repository root, a development build can then run:

```sh
cmake -S edge -B edge/build -DCMAKE_BUILD_TYPE=Release
cmake --build edge/build --parallel
ctest --test-dir edge/build --output-on-failure
```

For live observation, configure the ROS 2 and CPU inference options described in
[the critic deployment workflow](../models/critic/CLIENT_WORKFLOW.md).
Robot rollout uses the published installer rather than a source installer.

## Simulator compatibility binding

The simulator currently imports a flattened, committed Python binding at
`robot/sim/sim_rig/schemas/enforcement_pb2.py`. Refresh it deliberately when its
schema changes, with `grpcio-tools` installed in the selected Python environment:

```sh
./schemas/generate_sim_bindings.sh
# Or: make -C robot/sim proto
```

Set `PYTHON=/path/to/python` to select that environment. This helper only updates
the simulator binding; it does not generate obsolete C++ or TypeScript stubs.
Review and commit the generated simulator file so cloud image builds use the
intended version.

Field numbers are permanent: never reuse or renumber them. Schema descriptors
embedded in recorded MCAP files must remain decodable by compatible consumers.
