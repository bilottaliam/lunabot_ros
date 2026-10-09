#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

# Detect Ubuntu codename and map to ROS distro
UBUNTU_CODENAME=$(lsb_release -cs)
case "${UBUNTU_CODENAME}" in
    resolute) ROS_DISTRO="lyrical" ;;
    *)
        echo "ERROR: Unsupported Ubuntu release '${UBUNTU_CODENAME}'. Only resolute (26.04) is supported."
        exit 1
        ;;
esac

echo "Detected Ubuntu ${UBUNTU_CODENAME} - using ROS 2 ${ROS_DISTRO}"

install_ros_dependencies() {
    echo -e "\n------------------------ Install ROS Dependencies Using rosdep ------------------------ \n"
    cd "${WORKSPACE_DIR}"
    sudo rosdep init 2>/dev/null || true
    rosdep update
    # libpointmatcher is built by install_pointmatcher; velodyne is only used by rtabmap_examples
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "libpointmatcher velodyne"
}

install_camera_dependencies() {
    echo -e "\n------------------------ Install Camera Dependencies ------------------------ \n"
    sudo apt install -y ros-${ROS_DISTRO}-realsense2-*
}

install_git_dependencies() {
    echo -e "\n------------------------ Install Git Dependencies and Pull LFS------------------------ \n"
    cd "${WORKSPACE_DIR}/src/lunabot_ros"
    sudo apt install git-lfs -y
    git lfs pull
}

# Builds a CMake project from git and installs it to /usr/local
build_from_source() {
    local URL="$1" DIR="/tmp/$(basename "$1" .git)"; shift
    sudo rm -rf "${DIR}"
    git clone --depth 1 "${URL}" "${DIR}"
    cmake -S "${DIR}" -B "${DIR}/build" -DCMAKE_BUILD_TYPE=Release "$@"
    cmake --build "${DIR}/build" -j"$(nproc)" || { echo "ERROR: ${URL} build failed."; exit 1; }
    sudo cmake --install "${DIR}/build"
    sudo ldconfig
}

install_sparkcan() {
    echo -e "\n------------------------ Install sparkcan ------------------------ \n"
    # The sparkcan PPA has no resolute build
    build_from_source https://github.com/grayson-arendt/sparkcan.git
}

install_pointmatcher() {
    echo -e "\n------------------------ Install libnabo and libpointmatcher ------------------------ \n"
    # Used by RTAB-Map ICP (Icp/Strategy 1), not released for Lyrical
    sudo apt install -y libboost-all-dev libyaml-cpp-dev
    build_from_source https://github.com/norlab-ulaval/libnabo.git \
        -DLIBNABO_BUILD_PYTHON=OFF -DLIBNABO_BUILD_TESTS=OFF -DLIBNABO_BUILD_EXAMPLES=OFF
    # Boost.System is header-only since Boost 1.89
    git clone --depth 1 https://github.com/norlab-ulaval/libpointmatcher.git /tmp/libpointmatcher_src
    sed -i 's/ system / /' /tmp/libpointmatcher_src/CMakeLists.txt /tmp/libpointmatcher_src/libpointmatcherConfig.cmake.in
    cmake -S /tmp/libpointmatcher_src -B /tmp/libpointmatcher_src/build -DCMAKE_BUILD_TYPE=Release \
        -DPOINTMATCHER_BUILD_EXAMPLES=OFF -DPOINTMATCHER_BUILD_EVALUATIONS=OFF -DBUILD_TESTS=OFF
    cmake --build /tmp/libpointmatcher_src/build -j"$(nproc)" || { echo "ERROR: libpointmatcher build failed."; exit 1; }
    sudo cmake --install /tmp/libpointmatcher_src/build
    sudo ldconfig
    rm -rf /tmp/libpointmatcher_src
}

install_livox_sdk() {
    echo -e "\n------------------------ Install Livox SDK2 ------------------------ \n"
    build_from_source https://github.com/Livox-SDK/Livox-SDK2.git
    echo "Livox SDK2 installed successfully"
}

install_gui_dependencies() {
    echo -e "\n------------------------ Install GUI Dependencies (PyQt5 and OpenCV) ------------------------ \n"
    sudo apt install -y python3-pyqt5 python3-opencv
}

main() {
    install_camera_dependencies
    install_git_dependencies
    install_sparkcan
    install_pointmatcher
    install_livox_sdk
    install_gui_dependencies
    install_ros_dependencies
}

main
