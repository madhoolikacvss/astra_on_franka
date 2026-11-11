# astra_on_franka

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
tag 1 axes, track cube tag 0's center, and retain unclipped board normalization.
There is no calibrated tag-to-robot-base transform or grasp TCP yet.

Camera shutdown now allows a shared 10-second deadline (`shutdown_timeout_s` in
camera stream settings). Per-camera `stopping`/`closed` phases and stop duration
help distinguish capture delays from SDK shutdown delays. This is diagnostic
hardening; a hardware shutdown fix still needs verification on the lab system.
