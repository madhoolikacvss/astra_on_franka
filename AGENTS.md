# Hardware motion rules

## Current policy: supervised velocity-limited trial

The user selected the franka-toy-style velocity policy. The temporary blanket
model-preflight blocker is removed; the normal operator execution gates remain.
There is no custom per-move joint-excursion cap. The session excursion cap is
`null` in config/agent_bridge.yaml (disabled). If an operator configures a finite
session cap, it is checked before commands and throughout motion.
Measured joint speed must not exceed 0.10 rad/s, matching franka-toy's default
velocity setting. Our commanded joint speed remains capped at 0.02 rad/s and
Cartesian speed at 5 mm/s; acceleration, jerk, pose corridor, freshness, stop,
and 30 s per-command timeout remain unchanged. Cumulative session translation,
rotation, joint excursion and command-count caps are disabled (null).
This is not the franka-toy controller copied wholesale. No model-based joint
trajectory or full-body collision check is implemented. Keep operator supervision.

## Local handoff: supervised visual cube approach

The +X trial completed successfully in session_20261117T223858_047148Z_edaa7388:
measured displacement was approximately [+50.043, +0.001, +0.035] mm.
The current task is to move the gripper toward the cube until the estimated
gap between the nearest gripper surface and cube surface is roughly 5 cm.
Use fresh images and observations; a rough visual estimate is sufficient, not
an exact distance measurement. This target refers to the cube, not the robot base.
This authorizes bounded arm approach, not contact, closing the gripper, grasping,
or pick-and-place. The human supervises and starts/stops the service.

### Operator setup

Use the Linux workspace `/home/madhoolika/workspace/astra_on_franka` and conda
`reset`. Stop competing hardware owners and reset controller/command.json to
`{}` before starting a new service. Preview with:
`python scripts/agent_controller.py --mode live-preview`.
The usual operator gates, realtime scheduling and clear-path confirmation are
still required for `--mode live-execute --confirm-clear-path`. Do not mark
unverified gates true or let the agent edit configurations. Current local defaults
remain disabled. A preview is a proposed step from the real pose, not simulation
of motion. Stop the service with Ctrl-C when finished.

### Agent approach procedure

1. Read this file, then read only data/. Write only controller/command.json.
   Require matching session IDs, a running live service, heartbeat and publication
   age under one second, no fault/observation_error, not busy, and
   `observations_stable: true` plus `ready_for_command: true` in status.json.
2. Inspect top-down, wrist and side preview_file images. Use arm pose and the
   direction of visible image changes to choose the next robot-base axis step:
   +X forward, +Y left, +Z up. Do not assume camera image axes equal robot axes.
   Use a visible tag position as supporting evidence, not an unquestioned target.
3. Choose translations in any direction (tx+/tx-/ty+/ty-/tz+/tz-) and rotations
   about any base axis (rx+/rx-/ry+/ry-/rz+/rz-) as needed to approach the cube.
   Descent and changes of orientation are authorized; there is no fixed-height
   or fixed-orientation requirement. Choose the sequence from fresh visual
   observations without asking permission for each successful next step.
   Maintain clearance for the whole gripper and wrist camera throughout each
   translation or rotation, including the volume swept by the fingertips.
   Never touch the tabletop or cross the configured workspace boundaries.
   The point guards alone do not establish full-tool clearance; use the fresh
   views and pause when that clearance cannot be assessed.
   Prefer 10 mm steps, reducing to 5 mm near the estimated 5 cm gap. At most
   50 mm is allowed per command. Stop when roughly 5 cm away; do not chase an
   exact distance or make repeated corrections after reaching that rough gap.
   Rotations use the existing 1-degree presets, one observed step at a time;
   omit translation_m for rotation requests. There is no cumulative rotation cap.
   EE pose is not the fingertip pose; custom tool offsets are not fully calibrated.
4. Submit one request with fresh IDs, for example:
   {"schema_version":1,"request_id":"approach_01_UNIQUE","session_id":"CURRENT",
    "observation_id":"FRESH","action":"arm_step",
    "arguments":{"preset":"tx+","translation_m":0.01}}
   Omitting translation_m defaults to 50 mm, so specify it explicitly for approach.
5. Wait for the matching terminal response. After completed, wait for stable
   observations again and inspect all views before deciding another step. In
   preview, report what was proposed; do not assume the pose changed.
   Consistent qualitative evidence of approaching the cube is sufficient: an
   individually visible or measurable gap reduction after every 10 mm step is
   not required. The operator confirmed the previous +Y step moved toward the
   cube in that setup; use this as supporting evidence, not a universal direction
   rule. Continue while fresh views support the direction and clear tool path.
   Pause if evidence contradicts the direction or clearance becomes uncertain.
6. Continue one observed step at a time until the approach objective is reached
   or the operator stops the task. There is no session-start displacement or
   rotation cap and no 20-command budget. Per-command sizes, speed limits and
   physical workspace guards still apply. Do not change files, limits or restart
   to bypass a rejection. Do not stop solely because reaching the objective
   requires descent or a change of orientation.
7. Reject/fail/controller fault, stale camera or arm/gripper data, unexpected
   movement, or operator stop ends motion. Do not retry a failed command or
   replan around a guard. Report achieved displacement and remaining uncertainty.

### Occlusion is not a sensor fault

A missing AprilTag detection alone does not make observations invalid and does
not end the task. Inspect the other fresh views; the cube may still be visible.
Use its visible appearance and the last known location to infer a next small
step when the target direction and full tool path remain clear. Label that
reasoning as visual inference, not a fresh measured 3D position. No calibrated
camera-to-base target is assumed for the wrist or side view.

`observation.cube` always reports the current detector result.
`observation.cube_context.last_seen` is historical, with source timestamps and
age; never treat it as current or assume the cube stayed still after contact.
A last-seen location may support a small approach step if continuity and
clearance are visible. Do not descend toward an obscured contact region based
solely on memory. If the cube and relevant path are unclear in every view, wait
for a clearer view or ask the operator; this is a pause for uncertainty, not a
permanent abort merely because a tag disappeared. A small viewpoint adjustment
is allowed only when its path is visibly clear and all controller guards pass.

### Recordings and limits

Each live session saves `data/agent_bridge/session_<timestamp>_<id>/side_view.mp4`
and `side_video_frames.jsonl` in that same session directory. Video uses
existing side RGB observations resampled to 10 fps, not every native 30 fps frame.
Repeated frames and source ages are logged; a frozen video image is not fresh
sensor evidence. The MP4 is finalized on normal shutdown; use JPGs for live
agent perception. The five-frame-set JPG/NPZ buffer remains bounded, while the
compressed session video grows with duration. Historical sessions are retained.

Commanded Cartesian speed remains 5 mm/s, joint speed 0.02 rad/s, and the measured
joint ceiling 0.10 rad/s. Workspace point guards, freshness and stop handling
remain enabled. These are not full-arm/tool-volume collision checks. No gripper
commands are part of the approach trial.

## Astra robot-operation role

These restrictions apply when operating the robot through the bridge. They are
not permission for an operating agent to become a developer or administrator.
An explicit human software-development request is a separate maintenance role;
stop the controller before maintenance. During robot operation:

- Read this instruction file once. Operational observations/results may ONLY be
  read from `data/`. Do not inspect or modify code, configs, tests, other repos,
  credentials, logs outside data, or environment settings.
- The ONLY permitted write is replacing the CONTENTS of the existing
  `controller/command.json`. This is a JSON inbox, never Python code.
  Do not create, delete, rename or edit any other file, including this AGENTS.md.
  Do not change file paths, permissions, ownership, configurations, or limits.
- Do not start/stop services, run hardware scripts, access FCI/Desk/serial devices,
  install packages, open another robot connection, or execute shell/network
  commands to bypass the bridge. An operator starts the trusted service.
- Read `data/agent_bridge/status.json` and `latest.json`. Match their session IDs;
  require a recent heartbeat, no fault, no observation_error and busy=false.
  Read referenced images/observations only inside data. Replay is historical data,
  not evidence of the current robot position. Live-preview never causes motion.
- Submit ONE request, then wait for its matching response in
  `data/agent_bridge/response.json`. `accepted` means pending, not completed.
  Never queue by rapidly overwriting the inbox. Never reuse a request_id.
  Never automatically retry a rejected/failed request or replan around a guard.
- Copy session_id and observation_id from a fresh latest.json. No extra JSON
  fields are allowed. Normal schema:
  {"schema_version":1,"request_id":"unique_id","session_id":"from_latest",
   "observation_id":"from_latest","action":"arm_step",
   "arguments":{"preset":"tz+"}}
- Allowed arm presets: tx+/tx-/ty+/ty-/tz+/tz- (5-50 mm in base axes via optional translation_m; default 50 mm),
  rx+/rx-/ry+/ry-/rz+/rz- (1 degree about base axes through configured EE).
  Only translation_m may customize a translation step within 0.005-0.050 m.
  Do not request free-form coordinates, velocity, force, raw joints, code or paths.
- Gripper request uses action="gripper_set", arguments={"closed_fraction":0.5}.
  Choose a target in [0,1] within 0.10 of the fresh observed closed_fraction.
  0=open, 1=closed. Do not attempt object grasping until separately authorized.
- Stop request uses action="stop", arguments={}, observation_id=null, a fresh
  unique request_id and current session_id. It is allowed while busy or stale;
  stop latches the session and requires operator restart. A file stop is not an
  emergency stop and may be delayed; ask the operator to use the physical stop
  when necessary.
- Never report a preview or guard pass as executed movement or collision safety.
  The current guard covers EE/flange points, not the complete arm/tool. Do not
  claim that all physical arm motions are contained by the present checker.

OS permissions/sandboxing must enforce these restrictions. This document is not
a security boundary. The operating agent must not share the service's privileges.

## Maintenance and hardware-test rules

- Default to offline or live-observation preview. Do not execute robot/gripper
  movement merely to test code. Physical execution needs explicit user direction.
- Keep workspace motion_enabled false during development. Never enable it to
  make a test pass or bypass a preflight rejection.
- The supervised motion-test script permits one preset only, not an autonomous
  sequence. Never increase displacement, velocity, acceleration, jerk, or guard
  thresholds without explaining and reviewing the change with the user.
- Use a single shared Franky Robot for motion and arm observations. Never launch
  a second arm recorder alongside the motion test.
- On stale/invalid state, boundary violation, fault or timeout, stop the commanded
  motion, report the reason and do not retry, recover, home, or retreat automatically.
- The current guard checks configured EE and flange points, not complete arm/tool
  volumes. Never describe a preview pass as collision-safe. Physical path inspection
  is required for the supervised test. Keep Franka's collision defaults unchanged.
- Preserve raw logs and measurement provenance. Approximate registration residuals
  are not a guaranteed error bound. Do not fabricate missing tool geometry.
- Use fake hardware tests for motion orchestration. Test that preview sends no
  commands and faults request stop before considering lab execution.

Observation failures and recoveries are retained in session observation_events.jsonl.
Rejected commands are saved as rejection_*.json with the current observation error.
Status and summary preserve last_observation_error after recovery/shutdown; this
historical field does not replace the current observation_error readiness check.
Removing cumulative session caps does not expand physical workspace boundaries or authorize contact/grasping.
