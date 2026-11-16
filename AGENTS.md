# Hardware motion rules

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
- Allowed arm presets: tx+/tx-/ty+/ty-/tz+/tz- (5 mm in base axes),
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
