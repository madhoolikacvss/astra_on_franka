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

## Local handoff: supervised visual cube pick-up

The +X trial completed successfully in session_20261117T223858_047148Z_edaa7388:
measured displacement was approximately [+50.043, +0.001, +0.035] mm.
The user reports the approach succeeded on the lab computer; that latest session
has not been reviewed in this checkout. The current task is to approach, align
the two fingers around the 60 mm cube, grasp it, and lift it until the cube's
bottom surface is at least 50 mm above the tabletop. This is cube clearance,
not EE height or a 50 mm gripper-to-cube gap.
Rough visual estimates are sufficient; use fresh observations throughout.
This authorizes opening/closing the gripper and intentional finger contact with
the cube, followed by an observed upward lift. Tool contact with the tabletop,
lateral transport after grasping, and placing/releasing are not authorized.
The human supervises and starts/stops the service.

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

### Agent approach, grasp and lift procedure

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
   Prefer 10 mm steps, reducing to 5 mm near the cube. At most 50 mm is allowed
   per command. The previous roughly 5 cm gap is now an alignment checkpoint,
   not the stopping goal. Continue to the grasp position when alignment and
   tabletop clearance are visible. If the minimum 5 mm step would overshoot
   available clearance, pause and report rather than taking that step.
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
6. Continue one observed step at a time until the grasp-and-lift objective is reached
   or the operator stops the task. There is no session-start displacement or
   rotation cap and no 20-command budget. Per-command sizes, speed limits and
   physical workspace guards still apply. Do not change files, limits or restart
   to bypass a rejection. Do not stop solely because reaching the objective
   requires descent or a change of orientation.
7. Reject/fail/controller fault, stale camera or arm/gripper data, unexpected
   movement, or operator stop ends motion. Do not retry a failed command or
   replan around a guard. Report achieved displacement and remaining uncertainty.

### Grasping the cube

- Open the fingers enough to straddle the 60 mm cube before approaching its sides.
  Use only gripper_set with closed_fraction in [0,1], where 0=open and 1=closed.
  Each target must be within 0.10 of the fresh observed fraction. Submit one
  command at a time, then wait for completion and stable observations.
- Align the cube between the fingers using fresh views. Keep fingertips above
  the board; the tag center and EE origin are not fingertip/contact points.
  Orient with clear swept space before entering the grasp region.
- Keep the arm stationary while closing incrementally. Use smaller fraction
  changes near contact. Do not blindly command fully closed or increase the
  calibrated current settings. The existing 119 mm opening calibration makes
  a 60 mm opening approximately closed_fraction=0.496. This is a geometric
  guide, not a verified contact or grasp target.
- Inspect images and measured opening after each command. Stop closing when
  the cube appears held between the fingers; do not keep squeezing to prove
  a grasp. After a completed closing command and fresh stable observations,
  proceed with the observed lift below. Visual contact alone does not prove a
  secure grasp; do not report success until the cube visibly rises with the tool.
- The controller checks target opening, not grasp force. Contact can prevent
  reaching a target and cause `Gripper did not reach target; possible contact.
  No retry.` with torque disabled. This remains a failed command: stop and
  report it, never relabel it as success or bypass the timeout. Normal service
  shutdown also disables enabled gripper torque.

### Lift goal and completion

- After grasping, keep the grasp orientation and gripper target unchanged while
  lifting vertically with tz+ steps. Start with 5 mm to check that the cube rises
  with the fingers; continue with 5-10 mm steps after fresh observations confirm
  retention and clearance. Do not lift after a failed gripper command.
- The goal is at least 50 mm clearance between the cube's bottom and the tabletop.
  Use the side view, supported by other fresh views and depth where available.
  The 60 mm cube side is a useful visual scale. Aim slightly above the estimated
  50 mm threshold rather than claiming exact millimetre accuracy from images.
  EE displacement alone is insufficient: the cube must actually rise and remain
  held. Tag loss alone does not stop the task if fresh views show this clearly.
- Continue checking workspace bounds and clearance for the fingers, camera and
  carried cube. Current point guards do not check the carried cube's volume.
  Stop further movement if the cube slips, remains on the board, or its retention
  or path becomes unclear. Do not automatically squeeze harder or retry a grasp.
- Once the cube is visibly held at least approximately 50 mm above the tabletop,
  stop sending movement commands and report the estimated clearance and evidence.
  If the estimate cannot establish that threshold, report completion as unverified.
  Do not place or release the cube. The operator must support/secure the cube
  before stopping the service: shutdown disables gripper torque and can drop it.
  Do not disable or bypass fault/stop handling to preserve a hold; operator
  supervision is required throughout the lift.

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
remain enabled. These are not full-arm/tool-volume collision checks. Gripper
commands are authorized for the cube grasp and lift described above.

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
  0=open, 1=closed. The supervised cube grasp and lift above are authorized; other
  manipulation needs a separate task instruction.
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
Removing cumulative session caps does not expand physical workspace boundaries.
Only the cube-finger contact described in this grasp-and-lift task is authorized.
