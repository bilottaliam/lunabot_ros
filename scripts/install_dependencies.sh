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
    # libpointmatcher is built by install_source_dependencies; velodyne is only used by rtabmap_examples
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

install_source_dependencies() {
    echo -e "\n------------------------ Install sparkcan, Livox SDK2, libnabo, libpointmatcher ------------------------ \n"
    # No 26.04 / Lyrical packages for these; built at pinned commits (same script as the Dockerfile)
    sudo apt install -y build-essential cmake git libboost-all-dev libyaml-cpp-dev libeigen3-dev
    "${WORKSPACE_DIR}/src/lunabot_ros/scripts/install_source_deps.sh" /usr/local || exit 1
}

install_gui_dependencies() {
    echo -e "\n------------------------ Install GUI Dependencies (PyQt5 and OpenCV) ------------------------ \n"
    sudo apt install -y python3-pyqt5 python3-opencv
}

main() {
    install_camera_dependencies
    install_git_dependencies
    install_source_dependencies
    install_gui_dependencies
    install_ros_dependencies
}

main
