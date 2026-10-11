#!/bin/bash
# Builds and installs the dependencies that have no Ubuntu 26.04 / ROS 2 Lyrical package.
# Used by install_dependencies.sh and docker/Dockerfile.
#
#   install_source_deps.sh [PREFIX]   (default /usr/local)
#
# Each one is pinned to a commit that is known to build here, so a reinstall never pulls in
# upstream changes silently (sparkcan drives the real robot's motors).
set -euo pipefail

PREFIX="${1:-/usr/local}"
WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT

# Use sudo only when the prefix is not writable (not needed in Docker or for a user prefix)
SUDO=""
mkdir -p "${PREFIX}" 2>/dev/null || true
[ -w "${PREFIX}" ] || SUDO="sudo"

fetch() {  # fetch NAME URL COMMIT
    git init -q "${WORK}/$1"
    git -C "${WORK}/$1" fetch -q --depth 1 "$2" "$3"
    git -C "${WORK}/$1" checkout -q FETCH_HEAD
}

build() {  # build NAME [cmake args...]
    local dir="${WORK}/$1"; shift
    # CMake 4 rejects projects that declare a minimum below 3.5 (Livox-SDK2 does)
    cmake -S "${dir}" -B "${dir}/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
        -DCMAKE_PREFIX_PATH="${PREFIX}" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 "$@"
    cmake --build "${dir}/build" -j"${JOBS:-$(nproc)}"
    ${SUDO} cmake --install "${dir}/build"
}

# SparkMax CAN library for lunabot_teleop / lunabot_nav (its PPA has no 26.04 build)
fetch sparkcan https://github.com/grayson-arendt/sparkcan.git 11d2d13787fc32e508690926efc3c9749e58770e
build sparkcan

# Livox SDK2 for livox_ros_driver2
fetch Livox-SDK2 https://github.com/Livox-SDK/Livox-SDK2.git c0796f04c143143899c87a773d9f6b7136453c0b
build Livox-SDK2

# libnabo + libpointmatcher for RTAB-Map ICP (Icp/Strategy 1), not released for Lyrical
fetch libnabo https://github.com/norlab-ulaval/libnabo.git 0ce6ecbf1d845b9e1238f8e845a74a50cec04432
build libnabo -DLIBNABO_BUILD_PYTHON=OFF -DLIBNABO_BUILD_TESTS=OFF -DLIBNABO_BUILD_EXAMPLES=OFF

fetch libpointmatcher https://github.com/norlab-ulaval/libpointmatcher.git 62b347813e6aa4f0129457859e12a0371cc7a5da
# Boost.System is header-only since Boost 1.89 (26.04 ships 1.90) and no longer has a CMake package
for f in CMakeLists.txt libpointmatcherConfig.cmake.in; do
    grep -q ' system ' "${WORK}/libpointmatcher/${f}" || { echo "libpointmatcher ${f}: Boost patch no longer applies" >&2; exit 1; }
    sed -i 's/ system / /' "${WORK}/libpointmatcher/${f}"
done
build libpointmatcher -DPOINTMATCHER_BUILD_EXAMPLES=OFF -DPOINTMATCHER_BUILD_EVALUATIONS=OFF -DBUILD_TESTS=OFF

[ "${PREFIX}" = /usr/local ] && ${SUDO} ldconfig
echo "Source dependencies installed to ${PREFIX}"
