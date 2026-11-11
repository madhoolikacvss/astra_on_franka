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
