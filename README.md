# astra_on_franka

## Current supervised task: grasp the cube

Follow [AGENTS.md](AGENTS.md) for the operating-agent handoff. Approach and align
with the 60 mm cube, then open/close the fingers through the bridge to attempt a
grasp. The former 5 cm stopping gap is an alignment checkpoint. Lifting and
placing are not part of this task.

All existing speed, spatial boundaries, freshness, per-command and fault-stop
constraints remain unchanged. Never touch the tabletop. Gripper targets use
closed_fraction (0=open, 1=closed), with at most 0.10 change from the current
observation per command. Close in small observed increments with the arm still.
The agent may only read data and write controller/command.json.

The controller checks target opening, not holding force. A blocked target may
time out and disable torque: stop and report it, do not treat it as success or
retry past the fault. Report visual evidence without claiming verified holding
force. Shutdown disables enabled torque. Current EE/flange point guards do not
check the full tool volume; visible tabletop clearance and supervision remain
necessary. See AGENTS.md for the complete procedure.


Hardware integration project for the lab Franka Panda and custom Dynamixel gripper.

See [existing setup and data conventions](docs/existing_setup.md) for findings
from `gello-local` and `franka-toy`, source references, reuse caveats and remaining
calibration work. The report excludes policy and teleoperation workflows.

## Cartesian observer

On the lab Linux computer, from this repository:

```bash
conda activate reset
python scripts/record_observations.py
```

This opens one Franky connection and records one measured state snapshot. It
does not command motion, recover errors, change the tool frame, or access the
gripper/cameras. Franky's constructor applies its default collision thresholds,
matching the existing lab initialization. FCI must already be available.
The original filename `scripts/record_obervations.py` also works.

For a 10-second nominal recording, or continuous recording until Ctrl-C:

```bash
python scripts/record_observations.py --samples 150 --hz 15
python scripts/record_observations.py --samples 0 --hz 15
```

The requested rate is not a realtime guarantee. XYZ is printed for every sample;
joint positions, rotation matrix, and flange-to-EE transform are printed for the
first sample. All fields are saved for every sample. Outputs go into unique
`data/observations/arm_<UTC timestamp>_<id>/` directories:

- `metadata.json`: frames, units, package versions and clock conventions.
- `observations.jsonl`: measured joints/velocities, `O_T_EE`, `F_T_EE`, robot
  timestamp, host receipt timestamps, error state and mode.
- `summary.json`: completion/interruption/failure and number of valid samples.
- `rejected_sample.json` (transform-validation failures only): original matrices,
  determinant, orthogonality error and singular values from the rejected snapshot.
  Invalid samples are never written as valid observations or silently normalized.

`O_T_EE` maps configured EE coordinates into the robot base frame. `F_T_EE`
maps them into the flange frame. Matrices are stored as four rows with position
in meters; there is no quaternion-order ambiguity. The EE frame is not yet
verified as the custom gripper's grasp center. Do not compare this base-frame
XYZ directly with table measurements until the table-to-base transform is known.

Rotation validation uses absolute tolerance `1e-4` for both `R.T @ R - I`
and `det(R) - 1`. The lab's configured flange-to-EE rotation contains rounded
0.7071 coefficients, producing about `1.93e-5` error in both checks. Accepted
samples retain the original matrices and report these metrics under
`transform_validation`; no normalization or robot configuration change occurs.
The homogeneous bottom-row tolerance remains `1e-6`. These are numerical
validation tolerances, not physical workspace or contact-force limits.

Malformed/non-finite samples, invalid rotations, repeated/backward robot times,
and connection errors terminate recording with a failure summary. Robot error
flags are logged but never automatically cleared. A first sample cannot prove
freshness relative to an earlier sample; advancing robot time is also not a
measurement of end-to-end latency. Host monotonic time and robot time are
separate clocks. This diagnostic is not the later motion safety wrapper.

For initial verification, use a stationary, familiar pose established through
the existing lab procedure. Inspect the first sample, then a short recording
for finite, stable values, advancing robot timestamps and reported errors.
No home motion is performed by this script.

Offline verification (no Franky installation or hardware needed):

```bash
python -m unittest discover -s tests -v
```

API references: [Franky Affine](https://timschneider42.github.io/franky/api/generated/franky.Affine.html),
[RobotState](https://timschneider42.github.io/franky/api/generated/franky.RobotState.html),
[Duration](https://timschneider42.github.io/franky/api/generated/franky.Duration.html).

## Robot server version mismatch

The lab robot reported FCI server protocol 6; the generic PyPI Franky 2.0.0
wheel reported library protocol 10. An import or pip dry run does not verify
FCI compatibility. Use the maintainer's server-6 wheel index exclusively, to
avoid selecting the generic wheel again. Replace only Franky in reset:

```bash
conda activate reset
python -m pip install --force-reinstall --no-deps --only-binary=:all: \
  --index-url https://timschneider42.github.io/franky/whl/by-robot-server-version/6/ \
  'franky-control==2.0.0'
python -m pip show franky-control
python scripts/record_observations.py
```

The version pin permits a 2.0.0 local build suffix identifying libfranka.
No firmware or system libfranka change is requested. This repair still needs
verification on the lab computer. If pip cannot find a matching wheel for
Python 3.14/glibc 2.31, keep its full output; do not fall back to the generic
index or change Python automatically.

Source: [Franky installation instructions](https://timschneider42.github.io/franky/getting-started/installation.html).

## Three-camera RGB-D streaming

The camera recorder is independent of Franky: it never connects to the arm or
gripper. It selects the confirmed top-down, wrist and side serials from
`config/cameras.yaml`, without hardware resets or `/dev/video*` assumptions.
The `serial` field is the SDK device serial used for selection; `asic_serial`
retains the separately reported ASIC identifier. The fourth detected camera
is excluded. Inventory output and capture metadata report both identifiers.

On the lab computer, first enumerate the cameras, then capture a single set:

```bash
conda activate reset
python scripts/record_cameras.py --list
python scripts/record_cameras.py
```

For a short recording with a local desktop preview:

```bash
python scripts/record_cameras.py --samples 150 --hz 15 --preview
```

Omit `--preview` when running headless/SSH. Use `--samples 0` for continuous
recording until Ctrl-C (or q/Escape in the preview). Every recorded set includes
all three cameras. Compression and USB bandwidth may reduce the actual rate;
start with the short run before continuous recording. At 640x480, RGB plus two
16-bit depth arrays total about 6.45 MB per three-camera set before compression.

Requested capture is 640x480 at 30 fps for RGB8 and Z16 depth, matching the
resolution/rate used by the existing GELLO RealSense path. Settings are explicit
in YAML; unsupported profiles fail rather than silently switching resolution.
Color and native depth have separate intrinsics. Actual profiles, camera name,
firmware, USB descriptor, scale and factory depth-to-color extrinsics are logged.
No camera-to-robot calibration is implied by these factory intrinsics/extrinsics.

Each camera has its own acquisition worker and bounded latest-frame slot. The
recorder runs at a requested 15 Hz independently of the 30 Hz capture. Capture
pairs skipped between recorded sets are counted; this is intentional sampling,
not a claim of lossless capture. Streams run concurrently, but this does not
configure hardware exposure synchronization.

Each unique `data/cameras/cameras_<UTC timestamp>_<id>/` contains:

- `metadata.json`: settings, versions, units and clock conventions.
- `observations.jsonl`: per-camera serial, both stream frame numbers/timestamps
  and timestamp domains, host receipt timestamps/age, calibration, image file
  path, depth-valid fraction and skipped capture-pair count. Receipt spread
  reports the difference between earliest/latest host arrivals in each set;
  it is not exposure skew. Do not directly subtract hardware-clock timestamps
  from different cameras or from the robot's uptime.
- `frames/<view>_<index>.npz`: RGB uint8 `(H,W,3)`, native `depth_raw` uint16
  `(H,W)`, and `depth_aligned` uint16 on the color grid. Depth meters equal
  raw units times `depth_scale_m_per_unit`; zero indicates missing depth.
  Aligned intrinsics are recorded separately. RGB is not BGR.
- `summary.json`: completion, interruption or failure, and committed set count.
  A disk failure can leave image files without a corresponding JSONL row;
  only rows committed to JSONL represent complete recorded sets.

The preview shows RGB beside depth colored over a 0-2 m display range; this
display clipping never modifies saved depth. No image rotation is applied.

Freshness uses monotonic host receipt time (default 0.5 s maximum age).
Repeated frame numbers/timestamps do not refresh it. Backward counters or
changed timestamp domains fail explicitly; missing streams, disconnects,
camera timeouts and stale frames stop the recorder and produce a failure
summary. A newly received old buffered frame cannot be proven fresh solely
from its receipt time; exposure-age estimation requires clock calibration.
Calibration and freshness metadata do not constitute motion safety checks.

Both camera and cube recorders save `camera_diagnostics.json` on normal exit,
interruption and caught failures. It retains the last 16 frame-pair decisions per
camera with previous accepted/current counters, timestamp domains and timestamps.
Zero-counter pairs are discarded without replacing the last accepted frame or
refreshing its freshness timer. A later valid advancing pair may resume capture;
persistent zeros still cause startup timeout or runtime staleness. Nonzero
counter regressions, backward timestamps and domain transitions remain errors
with separate messages. This handles a possible transient zero reported in the
lab's depth stream without assuming that a real camera reset is harmless.

Startup now requires `startup_valid_pairs: 5` consecutive advancing pairs before
the first publication. Startup counter/domain transitions discard the pair and
restart warm-up; after publication they still fail. The original 10-second
startup deadline remains. Pairs with different timestamp domains or more than
`max_rgb_depth_skew_ms: 70.0` separation are discarded without refreshing
freshness. This responds to the observed startup depth counter sequence
3 -> 0 -> 1 and a 165 ms RGB/depth mismatch; it does not synchronize exposures.

The implementation follows the [RealSense alignment example](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py).

### RealSense GLIBC_2.32 import failure on Ubuntu 20.04

The distributed `pyrealsense2==2.59.1.11053` CPython 3.14 wheel was accepted by
pip but fails to import on this lab's glibc 2.31 host. A successful pip install
does not verify native library compatibility. Do not replace system glibc.

Build a local wheel against the host and active reset interpreter:

```bash
conda activate reset
# Install missing build prerequisites only (not a system upgrade):
sudo apt-get install build-essential cmake git pkg-config libusb-1.0-0-dev libssl-dev
bash scripts/build_realsense_local.sh
```

The helper uses upstream v2.59.1, a static SDK and the userspace RSUSB backend.
It does not install system SDK libraries, firmware, udev rules or kernel
patches; existing device access rules remain in use. It keeps source, wheel
and build logs in a unique ignored `.build/realsense-*` directory. It verifies
the built extension can import, then prints the explicit wheel-install command.
Run that command and retry `python scripts/record_cameras.py --list`.
The local package version is 2.59.1. The helper requires a network connection
for upstream build dependencies and has not been executed on the lab hardware.
If building fails, preserve its `build.log`; do not uninstall the old wheel first.

References: [official source build instructions](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/readme.md),
[upstream wheel definition](https://github.com/realsenseai/librealsense/blob/v2.59.1/wrappers/python/pyproject.toml).

## Standalone AprilTag cube observer

Adapted from the working reset-policy `track_apriltag.py` and `cube_tracker.py`.
It opens only the top-down camera, using the existing camera worker. Stop other
camera recorders/viewers before running. No arm, gripper, goal or combined
observation integration is performed yet.

```bash
conda activate reset
python -c "from pupil_apriltags import Detector; print('AprilTag import OK')"
# If missing, install the detector in reset:
# python -m pip install pupil-apriltags
python scripts/observe_cube.py
python scripts/observe_cube.py --samples 0 --hz 10 --preview
```

Use q/Escape or Ctrl-C to stop; omit preview when headless. Both tag 1 (fixed
reference) and tag 0 (cube) must be visible in the same frame. `config/cube.yaml`
preserves tag36h11, 50 mm tag size and the confirmed board bounds. The tracked
point is tag 0's center, not an independently calibrated cube center.

The observer uses the active color profile's intrinsics rather than applying
the legacy 1280x720 calibration to 640x480 images. Zero-distortion images need
no correction; Brown-Conrady uses OpenCV rectification; nonzero modified Brown
uses the SDK projection model to construct a cached rectification map. Other
nonzero models fail explicitly. First-map creation can cause an initial stale
result; subsequent frames reuse the map. This is a change in calibration source
from the old tracker, so compare against known physical board positions.

`T_reference_cube_tag = inverse(T_camera_reference) @ T_camera_cube_tag`.
XY normalization is unclipped: `(x+0.50)/0.43`, `(y+0.71)/0.59`. Values beyond
[0,1] remain visible and set `inside_board=false`. Yaw is radians about reference
Z; it is null if the Euler yaw is singular. Missing/rejected/duplicate required
tags or stale source frames yield `detected=false` and null pose coordinates.
No previous pose is held as a fresh measurement. Hamming corrections are
rejected by default; margin and pose-error values are logged but are not
calibrated uncertainty estimates. Occlusion or planar pose ambiguity still
requires physical verification before any robot control.

Unique `data/cube/cube_*` runs hold configuration metadata, per-frame JSONL,
first-frame and first-detection annotated PNGs, and a summary with detection
counts. Each record retains the source camera frame/timestamp/domain, intrinsics,
raw relative XYZ in meters, normalized XY, relative matrix, yaw, quality values,
and processing/age measurements. `completed` means the loop completed; check
`detected_samples` to know whether cube tracking succeeded.

Verify normalized XY at known board locations and cover tag 0 to confirm the
position becomes unavailable. The reference-tag frame is not yet transformed
into the arm's base frame. Original reset-policy files are unchanged.

## Combined arm, cameras and cube observations

From the activated lab `reset` environment, run:

```bash
python scripts/record_combined.py --samples 100 --hz 10 --preview
```

Close the standalone observers first. Omit `--preview` without a desktop;
`--samples 0` runs until Ctrl-C or q. This connects to the arm at `172.16.0.2`
(`--robot-host` overrides it), applies Franky's default initialization, and sends
no motion commands. Gripper observation and task-goal integration are not included.

Each unique `data/combined/combined_*/` contains metadata, a summary, final camera
diagnostics, JSONL observations, and three compressed RGB-D files per sample.
The cube detector consumes the exact top-down RGB frame saved in that sample;
`source_view`, `source_sequence`, `source_color`, and `source_file` identify it.
Intrinsics, depth scale, alignment metadata and original device clocks are retained.

`valid` requires fresh camera/arm receipt times at assembly, an arm without errors,
and a detected cube. Missing tags produce an invalid sample with absent coordinates;
no previous cube position is substituted. Board inclusion is reported separately by
`cube.inside_board`. This validity flag is not authorization to move the robot.
Timing includes receipt spread, signed arm-to-camera receipt offsets, ages after
processing, and persistence delay. Receipt times do not establish synchronized
exposures; the requested rate is best effort and disk compression may slow it.

Arm poses use robot-base meters and joint radians. Cube positions use reference
tag 1 axes and track cube tag 0's center, confirmed by the user's moving-cube
check on 2026-11-12. Legacy normalization bounds remain unverified in this frame;
normalized values must not be used for motion until board calibration is corrected.
There is no calibrated tag-to-robot-base transform or grasp TCP yet.

Camera shutdown now allows a shared 10-second deadline (`shutdown_timeout_s` in
camera stream settings). Per-camera `stopping`/`closed` phases and stop duration
help distinguish capture delays from SDK shutdown delays. This is diagnostic
hardening; a hardware shutdown fix still needs verification on the lab system.

## Four-corner normalization and fresh center check

Active cube normalization now uses the measured corners from
`board_corners_20261116T205146_051836Z_ee0ab320`, embedded in `config/cube.yaml`.
A planar homography maps reference-tag XY to unclipped board coordinates:
near-left (0,0), near-right (1,0), far-right (1,1), far-left (0,1).
Directions use the robot-base viewpoint. This fits the four measured tag-center
placement limits, not physical wall geometry. Legacy bounds are fallback only.
Z and yaw remain in the reference-tag frame; neither is transformed by the
2-D normalization. The mapping is for cube placement on the board, not lifted
cube geometry. Both standalone and combined observers use the same mapping.

Place the cube at the independently measured center of the captured placement
region (keep the same orientation and resting height), then run:

```bash
python scripts/observe_cube.py --check-center --samples 100 --hz 10 --preview
```

Press Enter in the terminal when placement is ready. The preview then streams.
Do not use normalized feedback to choose the physical center: that would only
check the mapping against itself. `center_check.json` records detection rate,
median, spread and error. Default diagnostic acceptance requires a completed
run with at least 95% detection and 95% of detected samples within 0.03 of
(0.5,0.5) on both axes. This is a placement consistency check, not a certified
accuracy threshold. The active configuration remains `center_verified: false`
until independent evidence is reviewed. No arm connection or motion is made.

## Offline reference-tag to robot-base registration

`config/registration_points.yaml` holds corresponding physical points measured
in reference tag 1 axes and actual robot-base XYZ, in meters. At least three
noncollinear fit points and two independently measured check points are required.
The fixed robot does not need to move, but a metrology datum tied to its actual
base origin and axes is required. Table edges and lab forward/right/up are not
that datum until their relationship is measured. Cube-tag centers are above the
board plane: do not pair them with tabletop Z or physical board corners.

The current corner captures supply reference-frame cube-tag coordinates, but no
matching robot-base coordinates. Unknown fields remain null and are rejected.
After filling measured pairs, run `python scripts/register_board.py --max-error-mm N`,
replacing N with an explicitly chosen residual tolerance in millimeters. The
script fits a rigid transform (no scaling), reports fit and independent-check
errors, and saves a candidate in a unique `data/calibration/registration_*/`.
A diagnostic pass does not enable motion or install the transform. A common
axis/origin mistake in all input points can pass residual checks, so measurement
provenance must be reviewed separately. No hardware connections are opened.

## Stage-1 offline goal checker

```bash
python scripts/check_goal.py --goal 0.5 0.5
```

U increases left-to-right and V near-to-far, viewed from the robot base.
The checker inverts the four-corner homography, estimates resting tag-center Z
using a least-squares plane through the captured corner XYZ measurements, and
applies the accepted approximate reference-to-base transform. Output is the
cube TAG CENTER, not a grasp TCP, cube geometric center, or motion command.

The user's 60 mm cube dimension is recorded in workspace configuration. The
50 mm margin is applied inside the polygon of measured cube-tag-center corner
placements, using signed perpendicular distances to its edge lines in reference
XY meters (not a fixed normalized-coordinate inset). These are already placement
limits: no additional 30 mm half-cube subtraction is applied to that polygon.
Physical cube/tool volume checks remain stage 2. Reported workspace distances
use the configured point limits, without assuming the board margin also applies
to walls. Table-surface collision and a minimum tool height are not checked.

Each invocation saves result and configuration snapshots in a unique
`data/calibration/goal_check_*/` directory. Exit code 0 means stage-1 point checks
passed, 1 means goal rejected, and 2 means invalid input/configuration.
`motion_authorized` is always false. No hardware is opened. The measured fit
residual is not treated as a guaranteed error bound. The full-arm collision scene,
reachability, grasp pose, swept tool volume and trajectories remain unchecked.

## Stage 2: static tool-envelope checker

`config/tool_geometry.yaml` describes conservative axis-aligned boxes in the
CONFIGURED EE frame for the gripper, adapter, and wrist camera. Bounds must
include cables and the gripper's full finger opening range. The 119 mm opening
is not a complete outer bound. Missing bounds, coverage confirmation, or padding
produce an incomplete result, never a pass. The user must choose additional
measurement padding; the 50 mm board goal margin is separate.

```bash
python scripts/check_tool_pose.py --observations data/observations/ARM_RUN/observations.jsonl
```

A combined observations JSONL is also supported. `--sample` selects a zero-based
row; `--holding-cube` additionally requires the configured-EE-to-cube-center
attachment transform and uses the recorded 60 mm cube dimensions. No rigid
attachment is assumed from camera-tag observation alone.

The checker transforms all eight vertices of each padded box and reports minimum
signed clearance to front/rear/left/right/ceiling and a conservative global
horizontal tabletop plane. All tool components currently use the gripper's
nonnegative forward boundary; the arm-link rear allowance belongs to the future
full-arm checker. Table contact is not specially permitted. Tool-to-tool and
arm self-collision, internal obstacles, motion paths and inverse kinematics are
not checked. Even a static envelope pass never authorizes motion.

Recorded matrices use the observer's 1e-4 rotation tolerance and 1e-6 homogeneous
row tolerance, without silently normalizing them. Box padding must account for
measurement/model uncertainty. Results and exact input snapshots are saved to
unique `data/calibration/tool_check_*/` directories. Exit 1 means incomplete or
rejected; exit 2 means malformed inputs. The script opens no hardware connections.

## Preview-first supervised motion test

`test_motion.py` implements ONE small preset per invocation. It defaults to
preview and never homes, retries, recovers errors, or commands the gripper.

```bash
# Live arm state preview (connects but sends no move/stop/dynamics commands):
python scripts/test_motion.py --preset tz+

# Fully offline preview from a saved arm or combined JSONL:
python scripts/test_motion.py --preset tz+ --observations data/observations/ARM_RUN/observations.jsonl
```

Presets `tx+`, `tx-`, `ty+`, `ty-`, `tz+`, `tz-` translate 50 mm in ROBOT BASE
axes. `rx+`, `rx-`, `ry+`, `ry-`, `rz+`, `rz-` rotate 1 degree about a base-axis
through the configured EE origin, NOT around the fingertips. One preset only;
no chained movement or return-to-start is scheduled. Raw measured matrices are
preserved; the target rotation is projected to the closest proper rotation
because the configured lab EE transform has rounded coefficients.

Execution additionally requires workspace `motion_enabled: true`, `--execute`,
and `--confirm-clear-path`. The configuration remains false in this checkout.
The clear-path flag means the operator has inspected clearance for the complete
arm, gripper, camera and cables and has the physical stop available. Do not run
another arm recorder: this script shares one Franky Robot with ArmObserver.
A live preview changes Franky's default collision settings at construction, as
existing lab observers do; it issues no motion, recovery, or gripper command.

Requested effective translation limits are 5 mm/s, 10 mm/s² and 50 mm/s³.
Requested rotation limits are 1 degree/s, 2 degrees/s² and 10 degrees/s³.
The script sets explicit dynamics caps and a 0.05 global factor. Requested joint
limits are 0.02 rad/s, 0.04 rad/s², 0.2 rad/s³; actual joint velocity is separately
observed. Default Franka collision thresholds remain unchanged. No MAX_DYNAMICS
or abrupt direct torque control is used. Native controller behavior on the lab
installation still needs verification; no hardware execution was performed here.

Before submission, both configured EE and flange POINTS are checked at 21
interpolated poses against workspace limits with a 50 mm inward guard. This
separate test guard is not a model of tool volume and does not enforce board
placement normalization. EE uses no-behind-base; flange uses the arm rear limit.
The floor is conservatively treated as the tabletop plane across the workspace.
Only Idle, stationary, error-free starts are permitted. Execution re-reads state
and rejects displacement since preview rather than executing an obsolete target.

During motion, an independent observer thread polls state at a requested 100 Hz.
The main thread checks a 100 ms observation watchdog while joining asynchronous
motion in 5 ms intervals. It requests robot.stop() on reader failure, stale or
repeated robot time, invalid values, robot errors, guard crossing, >2 mm departure
from the translation segment, excessive rotation,
>0.10 rad/s observed joint speed, >15 mm/s observed EE speed, or >3 degrees/s
observed EE rotation. A native Franky time reaction also stops the motion after
30 seconds. Completion requires ten subsequent observations, final position
within 1 mm, orientation within 0.3 degrees, and settled joints. Stop failures are
reported explicitly. There are no automatic corrective moves.

These are BEST-EFFORT software checks. Python polling/network stop latency and
braking distance are nonzero; stop is not a guarantee against crossing a physical
boundary. The margin is not a certified braking/uncertainty budget. This script
does not enforce complete link/tool geometry, self-collision, or interior
obstacles. It is a supervised test, not an autonomous safety controller.

Arm observations (joint state, poses, timestamps, faults) are retained in memory
during the bounded test and written to a unique `data/motion_tests/` run afterward,
so disk writes cannot block supervision. A hard process crash can lose this buffer.
RGB-D acquisition/tag detection is not performed in the control-monitor thread.
The existing camera-only recorder may be used separately; do not start the combined
recorder or another arm connection simultaneously. Final preview, dynamics and
summary records are included. Native time reaction is not a hardware watchdog.

Repository review: `franka-toy/pi05_franka/controller.py` uses a 0.10 dynamics
factor and 0.10 rad/s joint command cap; `reset_franka_home.py` scales velocity,
acceleration and jerk to 0.10. `gello-local/teleop_franka.py` uses 100 Hz, 0.35 rad/s
cap, 0.20 velocity smoothing, 0.25 dynamics and startup alignment checks. Its
hardcoded joint-position arrays were not copied as verified FR3 limits. Neither
lab path establishes this workspace's full-body collision geometry.

Franky API references used:
- https://timschneider42.github.io/franky/guide/robot.html
- https://timschneider42.github.io/franky/api/generated/franky.Robot.html
- https://timschneider42.github.io/franky/api/generated/franky.CartesianMotion.html
- https://timschneider42.github.io/franky/guide/reactions.html

Hardware-free regression checks:
`python -m unittest discover -s tests -p test_motion_preview.py -v`.

## Astra agent bridge

See [Astra bridge setup](docs/astra_bridge.md) for the operator-run file bridge,
restricted command protocol and deployment permissions. The agent writes only
`controller/command.json` and reads published data. Start in replay or live-preview;
all execution gates remain disabled. This is setup for constrained requests,
not a declaration that full-arm collision safety has been implemented.
