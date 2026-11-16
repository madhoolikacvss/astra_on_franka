# Hardware motion rules

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
