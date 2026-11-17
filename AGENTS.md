# Hardware motion rules

## Local handoff: supervised 50 mm +X trial

The human has requested a total 50 mm translation in robot-base +X (forward).
This is an arm-control trial, not a cube-reaching or pick-and-place task.
Submit exactly ONE `arm_step` request with preset `tx+`, translating 50 mm.
This replaces the earlier ten-step plan. Do not send ten requests.
Keep orientation unchanged; do not command the gripper, descend, grasp, return,
or substitute a different direction. The agent may inspect all three camera
views to assess the scene. Only the trusted controller sends hardware commands.
The operator reported five completed legacy 5 mm steps (+25.122 mm total X),
then a timestamp fault on step six. That is not a completed 50 mm trial.
Require `translation_step_m: 0.05` in the new session's latest snapshot before
previewing this revised single-command trial. Do not resume the faulted sequence.

### Operator setup (human/service administrator, not the operating agent)

- Work directly on the lab Linux filesystem at
  `/home/madhoolika/workspace/astra_on_franka`; use `conda activate reset`.
  Do not use a cloud-synchronized copy for live commands or observations.
- Stop other arm, gripper, and camera owners. Clear the full arm/tool travel
  region, keep people out of reach, supervise continuously, and have the
  physical stop accessible. No object should be held during this trial.
- Before starting the service, reset the existing `controller/command.json`
  contents to `{}`. Use a new service session for the physical trial.
- First run `python scripts/agent_controller.py --mode live-preview`.
  Ask the agent to submit ONE `tx+` preview and inspect its matching result.
  A preview does not move the arm or simulate cumulative movement: repeated preview
  requests would repeatedly propose a step from the actual stationary pose.
- Before live execution, verify a supervised arm hardware test,
  the gripper hardware test required by the current bridge, and OS isolation.
  Do not mark an unperformed test as passed. The current bridge requires both
  hardware-test gates even when the requested task is arm-only.
- Execution gates currently remain false. Only the operator may enable verified
  gates while the service is stopped: `autonomous_execution_enabled`,
  `os_isolation_verified`, `arm_hardware_test_passed`, and
  `gripper_hardware_test_passed` in `config/agent_bridge.yaml`, plus
  `motion_enabled` in both `config/workspace.yaml` and `config/gripper.yaml`.
  The operating agent must stop and report a closed gate, not edit it.
- Once those prerequisites actually hold, the operator starts
  `python scripts/agent_controller.py --mode live-execute --confirm-clear-path`.
  This also requires realtime priority. Configuration is loaded at startup.
  The operator confirms in chat that the physical trial is ready and supervised.
- At trial end, stop the service from its terminal and restore execution gates
  to false during maintenance. Do not leave an unattended executable bridge.

### Operating-agent procedure

1. Read this file, then only operational files under `data/`. Write only the
   contents of the existing `controller/command.json`; no save-by-rename.
   Do not run hardware scripts or change configurations to prepare the trial.
2. Read `data/agent_bridge/status.json` and `latest.json`. Require matching
   session IDs, a running service (not `status: closed`), `live: true`, matching
   modes, no fault or observation error, and both snapshots not busy.
   Require heartbeat and publication age at most 1 second before each request;
   wait for a fresh snapshot if necessary. Never use a saved closed-session file.
   `motion_authorized: false` in latest is deliberately always false: it means
   an agent cannot authorize execution. Mode and operator gates determine whether
   the service executes. Never modify this field or treat preview as execution.
3. Inspect the three referenced `preview_file` images. Require valid arm and
   gripper observations, Idle/error-free arm, and available fresh camera frames.
   Camera `host_age_ms` reflects capture time, not age when the agent reads it;
   use receipt/publication timestamps to assess freshness. If the view is unclear
   or the proposed path appears obstructed, stop and report to the operator.
4. Record the initial `observation.arm.ee_position_base_m` and session ID in
   chat/memory. The intended endpoint is `[initial_x + 0.050, initial_y, initial_z]`
   in metres. This is the configured end-effector origin, not fingertip position.
5. Copy the newest observation/session IDs into the request below. Replace the
   placeholder request ID with a unique ID for every submission. Send exactly
   one request, then poll data files; never overwrite a pending request.

   ```json
   {"schema_version":1,"request_id":"xtrial_01_UNIQUE","session_id":"COPY_CURRENT_SESSION","observation_id":"COPY_FRESH_OBSERVATION","action":"arm_step","arguments":{"preset":"tx+"}}
   ```

6. Wait for `response.json` with the same request AND session IDs. `accepted`
   is pending. In preview, `previewed` ends the single-request preview trial;
   report the proposed target and that no movement occurred. In live execution,
   only `completed` confirms the command finished successfully. Verify a new, idle,
   valid arm observation and inspect the images after each completed step.
   Check that measured X increased by approximately 50 mm, with no unexpected
   Y/Z or orientation change; report actual displacement, not just step count.
7. End the physical trial after the single completed command. Do not send a
   second command, correct residual error, or restart to bypass the 50 mm session
   displacement cap. If any guard rejects or interrupts the move, report the
   measured partial displacement and stop. The unchanged 0.05 rad per-move joint
   excursion guard may stop a 50 mm move before it reaches the target.
8. Any rejection, failed action, session change, lost observation, unexpected
   movement, or operator stop ends the sequence. No retry, alternate direction,
   automatic recovery, or retreat. If a motion is pending, send the stop request
   described below when possible; for immediate danger use the physical stop.
   Report initial/final measured XYZ, delta in mm, completed-step count, final
   response, and any rejection/fault in chat. Do not write a separate report file.

### Controller limits and evidence

- Per command: 50 mm translation; current Cartesian speed cap 5 mm/s,
  acceleration 10 mm/s², jerk 50 mm/s³. Do not override dynamics.
- Session: 50 mm displacement from initial EE position, 5 degrees rotation,
  0.10 rad joint excursion, 20 accepted commands. This trial permits only one
  translation and no rotations/gripper commands, despite the broader interface.
- Each move is monitored against fresh arm state and EE/flange point boundaries;
  sensor failures during a command trigger cancellation. Full arm/tool collision
  geometry is NOT enforced, so operator inspection remains necessary.
- Latest contains arm pose/joints, signed gripper encoder and closed fraction
  (0=open, 1=closed), all three camera image paths, and cube tag observations.
  Cube tag is 0, fixed reference tag is 1. Cube visibility is not the objective
  of this fixed-direction trial; loss of a tag is not permission to reach blindly.
- Read `data/agent_bridge/response.json` for command outcomes and the current
  `data/agent_bridge/session_*/` for metadata, inbox history, result files and
  images. Latest persists after shutdown; always pair it with running status.
- The bridge retains only the latest five complete frame sets per session: five
  JPGs and five RGB-D NPZs per camera (15 JPGs and 15 NPZs total). Older image
  paths expire; if a read fails, re-read latest and its current paths. A sixth set
  exists briefly while it is written. Historical session directories are not
  automatically deleted.
- Most recent supplied snapshot showed valid arm/gripper/cameras/cube with
  gripper encoder 3889 (almost fully open). This is historical evidence only.
  A prior fully closed position failed encoder validation; do not recalibrate or
  close the gripper as part of this arm-only trial.

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
- Allowed arm presets: tx+/tx-/ty+/ty-/tz+/tz- (50 mm in base axes),
  rx+/rx-/ry+/ry-/rz+/rz- (1 degree about base axes through configured EE).
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
