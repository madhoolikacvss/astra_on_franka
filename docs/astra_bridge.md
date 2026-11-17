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
no command queue and at most one command/s. The accepted-command count is logged
but has no configured session cap.
Arm presets are limited to 50 mm or 1 degree and check EE/flange guard boundaries.
Session translation, rotation and joint excursion are uncapped by default (null). A finite configured session cap
is monitored throughout motion. The former 0.05 rad per-move cap is removed.
Measured joint speed is limited to 0.10 rad/s, based on franka-toy; commanded
joint speed remains capped at 0.02 rad/s. No model-preflight blocker is active. Gripper target changes are at most 0.10 closed fraction.

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

Camera storage is a rolling buffer of five complete frame sets per session,
including five JPG previews and five RGB-D NPZs per camera. A sixth set exists
briefly during publication; older frame paths expire. Re-read latest.json if an
image disappears before opening it. Existing older sessions are not purged. Every distinct inbox content
is journaled; final command results and session config are saved. Frame storage is bounded per session; metadata/timing logs still grow.
Stop the preview session when testing is finished.

## Local tests

`python -m unittest discover -s tests -v` covers schema/injection rejection,
observation expiry, session replay, step bounds, stop while busy, motion fault
stopping, Dynamixel enable ordering, and an end-to-end replay mailbox exchange.
These tests use fake hardware and a mocked OS lock. Native Linux lock behavior,
permissions and actual hardware execution require lab verification.

Observation failures/recoveries are recorded in `observation_events.jsonl`;
`rejection_*.json` retains each rejected request outcome and the current sensor
error. Live/closed status and session summary include publication/failure counts,
last publication time, last observation error, and any terminal exception.
Current `observation_error: null` means recovery, not absence of historical failures.

## Isolated gripper diagnostic

Operator only: stop the bridge and all camera/robot/gripper recorders first.
Leave wiring, USB topology, baud rate and existing servo settings unchanged so
the workload comparison is meaningful. Run from the project root in `reset`:

```bash
python scripts/diagnose_gripper.py record --seconds 260 --baseline data/agent_bridge/session_20261117T212400_715744Z_b1342719
```

This opens only the gripper serial port and reads the same position/error
registers as the bridge. It never writes servo registers, changes torque/mode,
or connects to the arm/cameras. Each failed observation is recorded, followed
by the normal 100 ms gap, with no immediate retry. SDK timeouts are unchanged.
Results are stored in a unique `data/gripper_diagnostics/gripper_*/` directory:
individual reads, observations, metadata, summary and `comparison.json`.
The comparison reports per-register failure rates and latency percentiles;
a difference alone does not prove USB contention. Repeat under comparable
conditions to assess variability.

Arm timing now has separate budgets: up to 20 ms from the first duplicate
snapshot to obtain an advancing timestamp, and 100 ms total read/validation
duration (including lock wait). Backwards time is rejected immediately. The
motion monitor's 100 ms watchdog is unchanged; blocking SDK calls cannot be
preempted by these Python deadlines.

## Isolated Dynamixel worker

The bridge now spawns a separate process that exclusively opens and owns the
Dynamixel port. Camera processing and the SDK serial loop no longer share a
Python interpreter. The parent requests the same sequential register reads at
the existing bridge cadence; this is not a background cache or a sensor retry.
Successful observations retain the worker's acquisition timestamps and include
`worker_pid` and `ipc_round_trip_ms`. Register logs also identify the worker PID;
session metadata records `gripper_transport`.

The worker rejects motion/enable operations in preview. All live writes still
use the existing ManagedGripper limits and the bridge's command filters.
Each RPC has a 250 ms transport deadline (2 s for shutdown); a valid read must
still complete its full parent round trip within 100 ms. The transport deadline
does not make older samples valid or extend SDK packet timeouts. IPC timeout or
disconnect latches a fault, closes the channel and requests worker cleanup; no
automatic command retry or worker restart occurs. Worker cleanup disables torque
if that worker enabled it. A hung process or broken hardware connection can
prevent confirmation of torque disable and is reported as a cleanup error.

To assess isolation, stop the old bridge, copy the updates, reset the inbox to
`{}` as operator, then record about 260 seconds in live-preview with no commands:

```bash
python scripts/agent_controller.py --mode live-preview
```

Stop with Ctrl-C and compare the new session's `dynamixel_reads.jsonl`,
`observation_events.jsonl`, and camera timing logs with the earlier combined
baseline. Use the isolated worker in the normal combined bridge; do not run the
standalone gripper diagnostic alongside it.

The gripper worker ignores terminal SIGINT; the parent handles Ctrl-C, cancels
active work, and asks the still-running worker to close normally. Broken-pipe
errors from genuine worker failures remain visible rather than being suppressed.

`status.json` now includes `observations_stable`, `ready_for_command`,
`stable_observations_s`, `stable_observation_count`, and `readiness_reason`.
Readiness requires two continuous seconds and at least ten healthy live checks,
with no observation gap above 500 ms, no fault/busy state, a stationary Idle arm,
and fresh arm/gripper/camera timestamps. It resets on failures and shutdown.
Non-stop live requests are rejected until ready. Readiness is a live observation
condition, not execution authorization, a full-path check, or a guarantee that
the next read will succeed. Session IDs and heartbeat must still be checked.

## Visual cube approach and side video

Every live session now records `side_view.mp4` (mp4v, nominal 10 fps),
`side_video_frames.jsonl` (source timestamps, frame ages and repeated-frame flags),
and `side_video_summary.json`. Encoding runs in its own thread with one latest
frame reference, using the existing sampled side camera feed rather than opening
another pipeline. It does not capture every native 30 fps frame. Normal shutdown
finalizes the MP4. Five JPG/NPZ sets per camera remain retained; the video grows
with session duration. Encoder errors are reported instead of silently claiming
a recording exists. No full historical RGB-D archive is added.

Translation requests may specify `arguments.translation_m` from 0.005 to 0.050;
omitting it retains the 0.050 m default. Rotation presets cannot specify this
field. Dynamics, per-command step limits and all point guards remain.
The current task is a supervised approach above the cube, without contact
or gripper commands. Session length alone no longer limits displacement.

Missing AprilTags alone do not invalidate readiness. `cube` contains the current
detector result; `cube_context.last_seen` contains explicitly historical tag data,
its source timestamps, and age at assembly. The agent may use fresh alternate
views and visual inference during occlusion; it must not misrepresent an inferred
position as a measured robot-base coordinate or descend into an unseen contact
region. Stale camera data, faults and rejected motions retain their stop rules.
See AGENTS.md for the revised approach handoff.

Shutdown can drain an earlier read response left pending by an interrupted parent
before accepting the close acknowledgement. Other RPCs still require exact IDs;
this does not retry a command or suppress a failed shutdown.
