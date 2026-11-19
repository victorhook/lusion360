#!/usr/bin/env bash
# Build OpenMVS (dense reconstruction, meshing, refinement, texturing) from source
# into a user prefix — no sudo, no CUDA required.
#
#   tools/photogrammetry/build-openmvs.sh            # installs to ~/.local/opt/openmvs
#   PREFIX=/opt/openmvs tools/photogrammetry/build-openmvs.sh
#
# Lusion360 finds the binaries in $PREFIX/bin/OpenMVS, ~/.local/opt/openmvs/bin/OpenMVS,
# on PATH, or in $LUSION_OPENMVS_DIR.
#
# System packages it needs (Ubuntu/Debian; most CAD/dev machines have them already):
#   sudo apt install cmake build-essential git libeigen3-dev libopencv-dev \
#        libboost-iostreams-dev libboost-program-options-dev libboost-system-dev \
#        libboost-serialization-dev libjpeg-dev libpng-dev libtiff-dev libglew-dev
# CGAL, nanoflann (header-only) and VCGLib are fetched here; GMP/MPFR are not needed.
# COLMAP (camera solving) is separate:  sudo apt install colmap
set -euo pipefail

OPENMVS_TAG=${OPENMVS_TAG:-v2.3.0}  # v2.4 needs OpenCV >= 4.10 (JPEG XL); Ubuntu 24.04 ships 4.6
CGAL_TAG=${CGAL_TAG:-v6.0.3}
VCG_TAG=${VCG_TAG:-2023.12}
NANOFLANN_TAG=${NANOFLANN_TAG:-v1.6.2}
PREFIX=${PREFIX:-$HOME/.local/opt/openmvs}
SRC=${SRC:-$HOME/.cache/lusion360/openmvs-src}
JOBS=${JOBS:-$(nproc)}

mkdir -p "$SRC"
cd "$SRC"

fetch() { # url tag dir
  if [ ! -d "$3" ]; then
    git clone --depth 1 --branch "$2" "$1" "$3"
  fi
}
fetch https://github.com/CGAL/cgal.git "$CGAL_TAG" cgal
fetch https://github.com/cnr-isti-vclab/vcglib.git "$VCG_TAG" vcglib
fetch https://github.com/jlblancoc/nanoflann.git "$NANOFLANN_TAG" nanoflann
fetch https://github.com/cdcseacave/openMVS.git "$OPENMVS_TAG" openMVS
(cd openMVS && git submodule update --init --depth 1 2>/dev/null || true)
# Util::formatTime passes the full buffer size at every append offset; with Ubuntu's
# _FORTIFY_SOURCE glibc aborts ("buffer overflow detected") the first time a progress
# line prints. Pass the remaining size instead.
sed -i 's/_sntprintf(buf+len, 128, /_sntprintf(buf+len, 128-len, /' openMVS/libs/Common/Util.h

# CGAL: configure-only "install" of the headers + CMake package
if [ ! -f cgal-install/lib/cmake/CGAL/CGALConfig.cmake ]; then
  cmake -S cgal -B cgal-build -DCMAKE_INSTALL_PREFIX="$SRC/cgal-install" -DCMAKE_BUILD_TYPE=Release \
        -DCGAL_DISABLE_GMP=ON -DWITH_CGAL_Qt6=OFF -DWITH_demos=OFF -DWITH_examples=OFF
  cmake --install cgal-build
fi
# nanoflann: header-only, installed next to CGAL
if [ ! -f deps-install/share/nanoflann/cmake/nanoflannConfig.cmake ]; then
  cmake -S nanoflann -B nanoflann-build -DCMAKE_INSTALL_PREFIX="$SRC/deps-install" \
        -DNANOFLANN_BUILD_EXAMPLES=OFF -DNANOFLANN_BUILD_TESTS=OFF
  cmake --install nanoflann-build
fi

cmake -S openMVS -B openMVS-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_PREFIX_PATH="$SRC/cgal-install;$SRC/deps-install" \
  -DCGAL_DIR="$SRC/cgal-install/lib/cmake/CGAL" \
  -DCGAL_DISABLE_GMP=ON \
  -DVCG_ROOT="$SRC/vcglib" \
  -DOpenMVS_USE_CUDA=OFF \
  -DOpenMVS_USE_PYTHON=OFF \
  -DOpenMVS_BUILD_VIEWER=OFF \
  -DOpenMVS_USE_OPENMP=ON \
  -DOpenMVS_USE_BREAKPAD=OFF
cmake --build openMVS-build -j "$JOBS"
cmake --install openMVS-build

echo
echo "OpenMVS installed to $PREFIX/bin/OpenMVS"
ls "$PREFIX/bin/OpenMVS"
