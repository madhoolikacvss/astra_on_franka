# Astra file bridge

The trusted operator process is `scripts/agent_controller.py`. Astra only writes
data requests to `controller/command.json`, not controller Python code. The
service owns hardware connections and snapshots its configuration at startup.
Configuration files are never read from request-supplied paths or reloaded on
agent demand. No eval, arbitrary import, shell command, or free-form motion is
accepted from the mailbox.

## Preview deployment first

Run on the lab Linux machine from the `reset` environment, with other arm and
camera recorders closed. The service takes a Linux process lock. VS Code should
access these live files directly on the lab host (for example via the existing
remote workspace), not via delayed cloud-folder synchronization.

Replay does not connect to any hardware:

```bash
python scripts/agent_controller.py --mode replay --observations data/combined/combined_20261111T233358_215960Z_671b506e/observations.jsonl
```

Replay uses the first recorded sample, explicitly labels it non-live and never
simulates successful movement. Existing combined records have no gripper state;
gripper requests are rejected in those replays. Use live preview for real gripper
observations, or synthetic records for protocol tests.

Live preview opens one shared Franky connection, a managed Dynamixel connection,
and all configured cameras. It does not enable torque or send motion commands:

```bash
python scripts/agent_controller.py --mode live-preview
```

Franky constructor initialization still applies its default collision thresholds,
as with the existing lab observers. Camera workers acquire RGB-D and track the
cube from the exact top-down frame. Images and NPZ files are published under
unique session folders. Do not start another arm/camera owner alongside this.
Press Ctrl-C in the operator terminal to close the service.

## Interface

| Path | Writer | Reader |
| --- | --- | --- |
| `controller/command.json` | Astra | Trusted service |
| `data/agent_bridge/latest.json` | Service | Astra |
| `data/agent_bridge/status.json` | Service | Astra |
| `data/agent_bridge/response.json` | Service | Astra |
| `data/agent_bridge/session_*/` | Service | Astra/operator |
| Code, configuration, AGENTS.md | Operator during maintenance only | Trusted service |

Latest snapshots carry session/observation IDs, arm and gripper observations,
camera metadata, cube position and image paths. Read current status as well:
latest.json can remain from the last successful snapshot when a sensor fails.
Responses carry request IDs. Accepted requests may finish as `previewed`,
`completed`, or `failed`; rejected requests explain why. A stop response means
`stop_requested`, not proof of an instantaneous physical stop. It sets a fault
and prevents further actions until operator restart.

Each request must contain exactly schema_version, request_id, session_id,
observation_id, action and arguments. The complete schema/examples are in
AGENTS.md. No agent-writable execute flag exists. Bounds, step sizes, durations,
dynamics, serial registers and paths cannot be overridden by a request.

Filters enforce session identity, unique request IDs, fresh published observations
(10 s maximum), no significant pose/gripper change since that snapshot, idle arm,
no command queue, at most one command/s, and 20 accepted commands per session.
Arm presets are limited to 5 mm or 1 degree and check EE/flange guard boundaries.
Translation from session start is capped at 30 mm, rotation at 5 degrees and
observed joint excursion at 0.10 rad. Individual motion supervision retains the
tighter per-move checks. Gripper target changes are at most 0.10 closed fraction.

Malformed JSON, duplicate keys, nonfinite numbers, unknown fields, oversized
requests, replay and out-of-range targets are rejected, never silently clipped.
The gripper's final raw encoder command is restricted to the established safe
encoder range. Its calibrated endpoint inset means requests 0/1 need not yield
an observed fraction of exactly 0/1.

## Live execution is disabled

All gates in config/agent_bridge.yaml remain false. Live execution requires
operator verification of OS isolation, supervised arm and gripper hardware tests,
the bridge enable flag, workspace/gripper motion flags, and explicit operator
clear-path confirmation. Astra cannot change these. A gate is an operator record,
not automatic proof of a successful test or of full-body safety.

The live adapter reuses the monitored motion-test path. Stop cancellation is
checked during execution; stale camera/arm observations cancel an active action.
Arm observation remains independent of image processing. Native Franky motion
timeout remains present. Gripper mode 5 must already be configured; no automatic
mode change occurs. Current is capped at the existing calibrated raw setting 45,
and current encoder position is seeded before torque enable. Setpoints ramp at
at most 0.10 fraction/s; this is not a guaranteed physical velocity limit. Failure
to reach the requested fraction within 5 s is a fault, not presumed grasp success.
Torque is disabled on stop/failure/shutdown; this can release a held object.
No object handling is authorized by this setup.

The present safety filter is NOT full-arm collision avoidance. It checks points
and motion deviation, not all links/tool volume, obstacles inside the workspace,
or safe grasp forces. There is nonzero observation, file-I/O, network and braking
latency. Do not enable unsupervised pick-and-place on the strength of these checks.
No automatic recovery, home, retreat or retry exists.

## Enforce permissions before using an operating agent

AGENTS.md alone cannot restrict a VS Code agent. Deployment must separate the
trusted service identity from the Astra/VS Code execution identity:

1. Service/operator owns code, configs, runtime directories and inbox directory.
   Astra must not own these files or have permission to chmod them.
2. Astra can read data and this instruction file, and write only the existing
   command.json contents. Its parent directory is not writable: prevent rename,
   deletion, symlink substitution or creation of executable files. Use a direct
   file write; editor save-by-rename cannot work with this permission model.
3. Astra has no sudo, service credentials, serial-device access or direct network
   access to the robot FCI/Desk. Restrict tools/terminal/network in its sandbox as
   well as filesystem permissions. A different username alone does not restrict
   network access. Do not run an unrestricted agent under the lab hardware user.
4. Trusted service can write data and use the camera, serial port, FCI network and
   realtime scheduling. Astra cannot start, replace or debug that service.
5. Operator verifies that unauthorized edits, hardware connections and alternate
   command paths are actually denied; then records os_isolation_verified. This
   repository has NOT installed lab OS permissions or changed account privileges.

Historical recordings and snapshots are preserved. Every distinct inbox content
is journaled; final command results and session config are saved. Sensor streams
consume disk space: stop the preview session when testing is finished.

## Local tests

`python -m unittest discover -s tests -v` covers schema/injection rejection,
observation expiry, session replay, step bounds, stop while busy, motion fault
stopping, Dynamixel enable ordering, and an end-to-end replay mailbox exchange.
These tests use fake hardware and a mocked OS lock. Native Linux lock behavior,
permissions and actual hardware execution require lab verification.
