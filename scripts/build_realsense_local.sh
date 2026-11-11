#!/usr/bin/env bash
# Build against this host's glibc and the active reset Python. No sudo/install.
set -euo pipefail

if [[ "${CONDA_DEFAULT_ENV:-}" != "reset" ]]; then
  echo 'Activate reset first: conda activate reset' >&2
  exit 1
fi
for tool in git cmake g++ pkg-config python; do
  command -v "$tool" >/dev/null || { echo "Missing build tool: $tool" >&2; exit 1; }
done
pkg-config --exists libusb-1.0 || {
  echo 'Missing libusb development files (Ubuntu package libusb-1.0-0-dev).' >&2
  exit 1
}

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$project_root/.build"
build_root="$(mktemp -d "$project_root/.build/realsense-XXXXXX")"
exec > >(tee "$build_root/build.log") 2>&1
echo "Building in $build_root"
python --version
getconf GNU_LIBC_VERSION
git clone --depth 1 --branch v2.59.1 \
  https://github.com/realsenseai/librealsense.git "$build_root/source"
git -C "$build_root/source" rev-parse HEAD > "$build_root/source_commit.txt"
python_path="$(python -c 'import sys; print(sys.executable)')"

# Static SDK + userspace USB backend: no system SDK install or kernel patches.
cmake -S "$build_root/source" -B "$build_root/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE="$python_path" -DPython_EXECUTABLE="$python_path" \
  -DBUILD_SHARED_LIBS=OFF -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_TOOLS=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_WITH_DDS=OFF \
  -DBUILD_ROSBAG2=OFF -DBUILD_GLSL_EXTENSIONS=OFF
cmake --build "$build_root/build" --target pyrealsense2 --parallel 2

# Upstream's wheel hook packages binaries; it does not compile them.
python - "$build_root" <<'PY'
from pathlib import Path
import shutil
import sys
import sysconfig
root = Path(sys.argv[1])
suffix = sysconfig.get_config_var('EXT_SUFFIX')
matches = list((root / 'build').rglob('pyrealsense2' + suffix))
if len(matches) != 1:
    raise SystemExit(f'Expected one built Python extension, found: {matches}')
package = root / 'source' / 'wrappers' / 'python' / 'pyrealsense2'
shutil.copy2(matches[0], package / matches[0].name)
(package / '_version.py').write_text('__version__ = "2.59.1"\n')
PY

# Verify the locally built module before packaging or touching installed wheels.
PYTHONPATH="$build_root/source/wrappers/python" python -c \
  'import pyrealsense2 as rs; print("Local import OK:", rs.__file__, rs.__version__); assert hasattr(rs, "pipeline")'
python -m pip wheel --no-deps --wheel-dir "$build_root/wheels" \
  "$build_root/source/wrappers/python"

echo 'Build complete. Install the local wheel into reset with:'
printf 'python -m pip install --force-reinstall --no-deps %q/pyrealsense2-*.whl\n' "$build_root/wheels"
echo 'Then: python scripts/record_cameras.py --list'
