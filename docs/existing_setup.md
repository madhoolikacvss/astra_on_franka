# Existing lab hardware and data conventions

## Confirmed lab decisions

User confirmation (2026-11-11): the hardware is unchanged from the working
repositories. Reuse the lab-specific interfaces and settings; resolve actual
code contradictions rather than requiring hardware re-identification.

- Goal current 45 is the calibrated grasp-force setting for this mechanism.
  Retain `grasp_current_raw: 45`; this does not label 45 as a force in newtons.
- Maximum opening 119 mm is verified calibration. Use the existing linear
  opening-to-width conversion; this does not specify the tool's outer geometry.
- The existing robot-side Dynamixel motor and register setup remain applicable.
- Disable gripper torque on shutdown.
- No custom Franka collision thresholds were found in the lab-specific code;
  the user confirmed using the existing Franky defaults for the observer.
  Future contact thresholds and geometric workspace enforcement are separate
  mechanisms: contact thresholds alone do not enforce a permitted region.
- Cameras are unchanged, but USB video-device paths may have changed.
  Top-down serial: `144223023540`; wrist serial: `223423020653`;
  side serial: `243623021110`. This supersedes the earlier side ID.
- Use the existing `reset` conda environment. No other process owns the
  cameras, robot or gripper, as confirmed by the user.
- User reports arm identifier `73681343`, described as "Franka Emika arm,
  Research 3". This suggests FR3 rather than the previously assumed Panda;
  confirm the model in Desk before selecting geometry or joint limits.
  This identification is not the robot system software version.
- Store observations in unique directories and refuse overwrites.
- Use `joint_positions_rad` and `gripper_closed_fraction` (`0=open`, `1=closed`).

## Arm connection and home

- Interface: `franky.Robot`; host default `172.16.0.2`.
- `current_joint_positions` is converted to a seven-element NumPy float64 array.
- The observation wrapper also returns `has_errors` and `is_in_control`.
- Joint angles are in radians, ordered as returned by Franky (seven arm joints).
- Home is `[0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]`, or
  `[0, -45, 0, -135, 0, 90, 45]` degrees.
- The home script previews by default, requires `--execute` for motion, checks
  realtime priority and robot errors, counts down three seconds, and uses
  velocity/acceleration/jerk relative factors of `0.10`. It calls `robot.stop()`
  in a `finally` block around the move.
- This home target is not a collision-checked route from an arbitrary start.

Sources: [robot observation](../../franka-toy/pi05_franka/robot.py),
[home script](../../franka-toy/reset_franka_home.py).

The source [environment file](../../franka-toy/environment.yml) declares
Python 3.11, NumPy 1.26, `franky-control==1.1.4`, `dynamixel-sdk==4.0.5`,
OpenCV and PyYAML. These are repository declarations, not proof of installed
lab versions. Capture the working Linux environment before changing packages.
Do not import the entire policy environment into this project.

## Robot-side custom gripper

### Reported Linux environment

The user supplied this inventory; it has not been tested from this Windows workspace.

| Component | Reported value |
| --- | --- |
| OS | Ubuntu 20.04.6 LTS |
| Kernel | `5.15.96-rt61` |
| Selected environment | `reset` |
| Selected Python executable | `/home/madhoolika/miniconda3/envs/reset/bin/python` |
| Python version in reset | `3.14.4`, Anaconda build, GCC 14.3.0 |
| dynamixel-sdk in reset | `4.0.5` |
| NumPy in reset | `2.5.2` |
| OpenCV in reset | `5.0.0` |
| franky-control | `2.0.0`, installed; import and Robot API inspection succeed |
| pyrealsense2 | `2.59.1.11053`, installation confirmed |
| PyYAML | `6.0.3`, installation confirmed |
| Host architecture / glibc | `x86_64` / `2.31`; pip selected a compatible manylinux_2_28 Franky wheel |
| Franka Desk system | `5.3.1`, build `81f87d2933` |
| Observed FCI server protocol | `6` (connection handshake) |
| System librealsense2, GL, udev rules, utils | `2.56.5-0~realsense.17053` |
| System librealsense2-dkms | `1.3.27-0ubuntu1` |
| System ros-noetic-libfranka | `0.9.2-1focal.20250426.004608` |

Python 3.14.7 was reported for the base interpreter, not for `reset`.
System library package versions do not establish which libraries a Python
binding uses. NumPy, OpenCV and dynamixel_sdk imports succeed in reset.
The user installed the three previously missing bindings in reset. The generic
Franky 2.0.0 import succeeds, but the hardware connection failed: server protocol
6 versus wheel library protocol 10. Run `arm_20261111T210832_622899Z_d5894441`
recorded zero valid samples. Python/OS wheel compatibility is distinct from
FCI protocol compatibility. Replace only Franky with a server-6 build; preserve
the robot firmware and reset interpreter. See README troubleshooting.
Subsequent run `arm_20261111T211949_705136Z_c6460805` reports
`franky-control==2.0.0+libfranka.0.12.1` and reaches state validation, resolving
the handshake failure. Its O_T_EE rotation failed the 1e-5 validation tolerance.
That run did not retain the matrix, so the cause is unconfirmed. The observer
now retains both transforms and numerical metrics in `rejected_sample.json`
on transform rejection; tolerance and original values are unchanged.

Follow-up `arm_20261111T212540_102195Z_b5bed84d` identifies the precision issue:
F_T_EE has rounded coefficients `0.707099974155426`, with determinant
`0.9999807469012083` and orthogonality error `1.92530987916939e-5`. O_T_EE
has the same determinant/error to numerical precision. The observer tolerance
was subsequently changed from `1e-5` to `1e-4`, preserving all original matrix
values and logging metrics on accepted samples. A regression test uses both
matrices from this run; malformed transforms remain rejected.

The configured EE is approximately 103.4 mm along flange +Z with a -45 degree
rotation about flange Z. This describes the current configuration, not a
verification that it matches the custom gripper's physical grasp center.

Run `arm_20261111T212834_579812Z_9a0da498` completed with one valid pose sample,
confirming connection and pose decoding, but not sustained streaming. It exposed
`has_errors: true` with `current_errors: "[]"`: the observer had used object
truthiness. The observer now aggregates the snapshot's named boolean error
properties and logs `error_flags`; the original run remains unchanged.
The system ROS libfranka package does not identify the library bundled in Franky.

Camera import subsequently failed: the installed pyrealsense2 binary requires
GLIBC_2.32, but the lab host has glibc 2.31. Camera enumeration has not yet run.
The README documents building a local wheel in reset, without changing system
glibc; a local source-build helper is provided but lab verification is pending.

### Cartesian observation API

Installed Robot API inspection confirms `state`, `current_pose`,
`current_cartesian_state`, `current_cartesian_velocity`, `current_joint_state`,
`current_joint_positions` and `current_joint_velocities`.

For recording, prefer one `robot.state` snapshot and extract its measured
`q`, `dq`, and `O_T_EE`, rather than reading separate convenience properties
and assuming they share a sample time. Preserve robot time when available,
plus host read-start/read-end timestamps.

The [Franky 2.0.0 state definition](https://raw.githubusercontent.com/TimSchneider42/franky/v2.0.0/include/franky/robot_state.hpp)
defines `O_T_EE` as measured configured end-effector pose in the base frame,
and `F_T_EE` as end-effector pose in the flange frame. These use Franky Affine
objects, not necessarily flattened libfranka arrays. Preserve transforms as
explicitly labeled 4x4 matrices; do not assume a quaternion order or reshape a
flat array without checking its representation. The configured EE frame is not
automatically the custom gripper fingertip/tool-center frame.

The [Franky 2.0.0 constructor](https://raw.githubusercontent.com/TimSchneider42/franky/v2.0.0/src/robot.cpp)
calls `setCollisionBehavior` during initialization. Therefore constructing
`Robot(host)` is not strictly configuration-read-only even if no motion is
commanded. Account for the lab's existing collision settings before deploying
an observation connection; do not describe initialization as having no writes.
The user has confirmed the default initialization. The implemented
[observer](../src/astra_on_franka/hardware/arm_observer.py) uses this path without
adding custom settings. See the [run instructions](../README.md#cartesian-observer).

Both [Franka-toy gripper](../../franka-toy/pi05_franka/gripper.py) and
[GELLO robot-side gripper](../../gello-local/gello/robots/franka_dynamixel_gripper.py)
agree on these settings:

| Setting | Value |
| --- | --- |
| Motor family described by wrappers | XM430; existing motor unchanged, confirmed by user |
| Adapter | `FT8ISG7I` |
| Linux device | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISG7I-if00-port0` |
| Motor ID | `10` |
| Baud rate | `4000000` |
| Packet protocol | `2.0` |
| Calibrated open / closed encoder values | `3892` / `-143` |
| Inset command open / closed values | `3832` / `-83` |
| Goal current | `45` raw units |
| Operating mode | `5`, described as current-based position mode |
| Normalized command convention | `0 = open`, `1 = closed` |

| Register used by code | Address | Access width |
| --- | --- | --- |
| Operating mode | 11 | 1 byte |
| Torque enable | 64 | 1 byte |
| Goal current | 102 | 2 bytes |
| Goal position | 116 | 4 bytes |
| Present position | 132 | 4 bytes |

Reuse these register choices for the unchanged lab motor. The `scan()` helper
can optionally record its model number for inventory; this is not a prerequisite
for adapting the working signed-position reader.

Position reads convert unsigned 32-bit values to signed values before
normalization. Negative targets are written using `raw & 0xFFFFFFFF`.

The wrapper's observed closed fraction is:

```text
clip(1 - (raw - (-143)) / (3892 - (-143)), 0, 1)
```

Commands instead interpolate between the inset endpoints:

```text
round(3832 + clip(closed_fraction, 0, 1) * (-83 - 3832))
```

Thus an open command reads approximately 0.0149 and a closed command 0.9851
under the observation mapping. Do not require exact 0/1 feedback for completion.
Neither wrapper proves grasp success or waits for target completion; successful
register writes only establish that the communication operation succeeded.

### Differences to preserve explicitly

- GELLO's `enable()` reads the position and writes a mapped position target
  before enabling torque. Franka-toy's version does not write a position target
  during enable. The GELLO mapping uses the inset range, so it is not an exact
  raw-position hold. Startup behavior needs hardware validation before reuse.
- Both wrappers disable torque on close. Preserve this user-confirmed shutdown
  behavior; report a communication failure if a disable write cannot be delivered.
- Goal current `45` is the user-confirmed calibrated grasp-force setting.
  No numerical force in newtons was supplied; keep the register value in raw units.
- [One-shot reader](../../franka-toy/tests/read_gripper_position_once.py) reports
  the opposite convention: `0 = closed`, `1 = open`. It estimates width as
  `119.0 * opening` millimeters. The user confirmed this calibration; adapt
  its convention as `width_mm = 119.0 * (1 - gripper_closed_fraction)`.
- [Connection test](../../franka-toy/tests/test_dynamixel_connection.py) uses
  different normalization endpoints, `120` and `3526`, and does not apply the
  same signed conversion. Reuse its ping concept, not its normalized output.

## Cameras

[Franka-toy camera code](../../franka-toy/pi05_franka/cameras.py) opens both
external `/dev/video4` and wrist `/dev/video10` color devices through V4L2,
requesting 1280 x 720 at 15 fps with YUYV. It converts BGR to RGB.
Those device paths are historical assignments, not stable camera identities.

The camera code pasted separately in this conversation differs: it opens only
the external serial `144223023540`, has wrist acquisition commented out, and
contains another unused identifier `310643060553`. The latest confirmed mapping
is top-down `144223023540`, wrist `223423020653`, side `243623021110`.
The old unused identifier is superseded. Select by serial instead of relying
on old `/dev/video*` assignments. These identities are recorded in
[camera configuration](../config/cameras.yaml); acquisition profiles remain unset.

[GELLO RealSense driver](../../gello-local/gello/cameras/realsense_camera.py)
provides an existing depth acquisition starting point:

- Uses `pyrealsense2`; can select a device by serial.
- Requests depth Z16 and color BGR8, both 640 x 480 at 30 fps.
- Returns RGB color `(H, W, 3)` and raw depth `(H, W, 1)`.
- Optional flip rotates both images 180 degrees.
- Does not return depth scale, intrinsics, extrinsics, frame timestamps or
  color-aligned depth. Raw depth values must not be assumed to be meters.
- Optional resizing uses OpenCV's default interpolation for depth; retain
  native depth for geometry, or use a deliberate resampling/calibration scheme.
- `get_device_ids()` resets devices, as does initialization without an explicit
  device ID. Do not use these paths as a non-disruptive inventory operation.

All three cameras having depth is user-provided setup information; the code
does not establish their exact models, actual supported simultaneous profiles,
or lab calibration. All three serial-to-view mappings are now user-confirmed.

## How arm state and poses are stored

The lab-specific paths store measured joint configurations, not measured
Cartesian tool poses. A seven-value joint array is not XYZ plus quaternion.

| Source | Storage | Relevant measured fields |
| --- | --- | --- |
| [Franka-toy recorder](../../franka-toy/pi05_franka/trajectory.py) | Per-observation `.npz` files | `joint_positions`, `gripper_position`, images, `timestamp_ns` |
| Same recorder | `actions.jsonl` | `observed_joint_positions`, `observed_gripper_position`, `timestamp_ns` |
| [GELLO recorder](../../gello-local/gello/data_utils/franka_demo_recorder.py) | `trajectory.npz` | `joint_position` `(N, 7)`, `gripper_position` `(N,)`, timestamps |

Franka-toy timestamps use `time.time_ns()` at recording time. GELLO stores
elapsed episode time and absolute host-monotonic camera receipt timestamps.
These are not hardware-synchronized exposure timestamps. Its recorder stores
joint arrays as float32, camera images separately as numbered JPEGs, and
metadata in JSON. Gripper values use the closed-fraction convention of the
robot-side wrappers. Policy/action payloads are not needed for Astra's initial
observation recorder.

The Franka-toy recorder deletes an existing destination directory on creation.
Do not copy that behavior into an observation logger; use unique run directories
and refuse accidental overwrites.

### Quaternion and generic-wrapper pitfalls

[Generic Pose helper](../../gello-local/gello/data_utils/conversion_utils.py):

- Constructor and `to_quaternion()` use `[x, y, z, qw, qx, qy, qz]`.
- Internal quaternion storage uses `[qx, qy, qz, qw]`.
- `to_44_matrix()` places translation in the last column.
- Euler and axis-angle conversion helpers normalize angles by pi; their values
  must not be interpreted as ordinary radians without conversion.
- These utilities do not establish the lab's base/tool calibration.

[Generic Panda wrapper](../../gello-local/gello/robots/panda.py) is a different
Polymetis setup. It returns an all-zero `ee_pos_quat` placeholder and incorrectly
uses joint positions as its `joint_velocities` field. Its constructor also
initiates robot/gripper motion. Do not use it for Astra hardware observation.

For Astra, explicitly label joint positions in radians and gripper closed
fraction. When Cartesian poses are implemented, store a frame ID, tool frame,
position in meters, explicitly named quaternion order, measurement timestamp,
and calibration version. Obtain poses from the verified robot API or calibrated
forward kinematics; never fill missing poses with zero placeholders.

## Geometry and safety information still missing

The [simulation Franka class](../../gello-local/gello/dm_control_tasks/arms/franka.py)
references a Panda model with a Robotiq 2F85 gripper. That is not a model of the
custom Dynamixel gripper, adapter and wrist camera.

The reviewed lab-specific code does not supply a calibrated full-arm collision
scene, custom-tool mesh, tool mass/inertia, wrist-camera transform, or measured
wall/table transforms. Do not treat joint limits or reduced dynamics as those
missing checks.

User-provided workspace measurements are left 440 mm, right 470 mm, forward
640 mm and maximum height 760 mm, with a 910 x 640 mm table and base center
outside its near edge. They remain measurements pending clarification of the
base-to-edge offset, height datum, axis orientation and rear boundary. They
should not yet be converted into executable Cartesian limits.

## Next implementation steps

1. Capture installed Linux dependency versions from the working environment.
2. Adapt the lab-specific Franky observer and Dynamixel signed-position reader
   into managed connections with timestamps, finite-value checks and explicit
   conventions. Keep motor enable separate from observation.
3. Enumerate cameras without resets, identify views, and acquire timestamped
   RGB-D with scale, intrinsics and transforms.
4. Record joint state, raw and normalized gripper state, camera metadata and
   images into unique runs. Include per-device freshness and errors.
5. Measure/calibrate tool and workspace geometry, then add path validation and
   local supervision before exposing motion skills to Astra.

The initial review added this report and its README link. Subsequent user
confirmations updated the report and camera identity configuration. No hardware
drivers have been executed by this review.
