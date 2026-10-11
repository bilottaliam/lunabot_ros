FROM osrf/ros:lyrical-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000
ARG RENDER_GID=992

# Dependencies and tools for building and running the Lunabot ROS workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git git-lfs sudo curl \
    software-properties-common python3-pip python3-rosdep python3-colcon-common-extensions \
    python3-pyqt5 python3-opencv python3-serial \
    fonts-ubuntu fonts-noto-core fonts-dejavu-core \
    libgl1-mesa-dri libgl1 libegl1 mesa-utils \
    libboost-all-dev libyaml-cpp-dev \
    ros-lyrical-xacro ros-lyrical-joint-state-publisher \
    ros-lyrical-ros-gz ros-lyrical-gz-ros2-control ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Same UI font as the Ubuntu desktop
RUN printf '%s\n' \
    '<?xml version="1.0"?>' \
    '<!DOCTYPE fontconfig SYSTEM "fonts.dtd">' \
    '<fontconfig>' \
    '  <alias><family>sans-serif</family><prefer><family>Ubuntu Sans</family><family>Ubuntu</family><family>Noto Sans</family></prefer></alias>' \
    '  <alias><family>monospace</family><prefer><family>DejaVu Sans Mono</family></prefer></alias>' \
    '</fontconfig>' > /etc/fonts/local.conf \
    && fc-cache -f

# SparkMax CAN library used by lunabot_teleop. The PPA has no resolute build, so build from source.
RUN git clone --depth 1 https://github.com/grayson-arendt/sparkcan.git /tmp/sparkcan \
    && cmake -S /tmp/sparkcan -B /tmp/sparkcan/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/sparkcan/build -j"$(nproc)" \
    && cmake --install /tmp/sparkcan/build \
    && rm -rf /tmp/sparkcan

# libnabo + libpointmatcher for RTAB-Map ICP (Icp/Strategy 1). Not released for Lyrical.
# Boost.System is header-only since Boost 1.89, so drop it from libpointmatcher's components.
RUN git clone --depth 1 https://github.com/norlab-ulaval/libnabo.git /tmp/libnabo \
    && cmake -S /tmp/libnabo -B /tmp/libnabo/build -DCMAKE_BUILD_TYPE=Release \
       -DLIBNABO_BUILD_PYTHON=OFF -DLIBNABO_BUILD_TESTS=OFF -DLIBNABO_BUILD_EXAMPLES=OFF \
    && cmake --build /tmp/libnabo/build -j"$(nproc)" && cmake --install /tmp/libnabo/build \
    && git clone --depth 1 https://github.com/norlab-ulaval/libpointmatcher.git /tmp/libpointmatcher \
    && sed -i 's/ system / /' /tmp/libpointmatcher/CMakeLists.txt /tmp/libpointmatcher/libpointmatcherConfig.cmake.in \
    && cmake -S /tmp/libpointmatcher -B /tmp/libpointmatcher/build -DCMAKE_BUILD_TYPE=Release \
       -DPOINTMATCHER_BUILD_EXAMPLES=OFF -DPOINTMATCHER_BUILD_EVALUATIONS=OFF -DBUILD_TESTS=OFF \
    && cmake --build /tmp/libpointmatcher/build -j"$(nproc)" && cmake --install /tmp/libpointmatcher/build \
    && ldconfig && rm -rf /tmp/libnabo /tmp/libpointmatcher

# Livox SDK2, required by livox_ros_driver2. Its CMakeLists predates CMake 3.5, which CMake 4 rejects.
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    && cmake --build /tmp/Livox-SDK2/build -j"$(nproc)" \
    && cmake --install /tmp/Livox-SDK2/build \
    && rm -rf /tmp/Livox-SDK2

# Workspace dependencies from the package.xml files. libpointmatcher (built above) and
# velodyne (rtabmap_examples only) have no Lyrical release, so rosdep skips them.
COPY . /tmp/deps_src/lunabot_ros
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/lyrical/setup.bash \
    && apt-get update && rosdep update \
    && rosdep install --from-paths /tmp/deps_src --ignore-src -r -y \
       --skip-keys "libpointmatcher velodyne" \
    && rm -rf /var/lib/apt/lists/* /tmp/deps_src

# Non-root user matching the host uid so files written into the mounted src stay yours.
# The render group is created with the host gid so /dev/dri/renderD128 is usable.
# Ubuntu 24.04+ images ship an "ubuntu" user on uid/gid 1000, which would collide.
RUN userdel -r ubuntu 2>/dev/null || true \
    && (getent group render >/dev/null && groupmod -g ${RENDER_GID} render || groupadd -g ${RENDER_GID} render) \
    && groupadd -g ${USER_GID} lunabot \
    && useradd -m -u ${USER_UID} -g ${USER_GID} -G sudo,video,render,dialout -s /bin/bash lunabot \
    && echo "lunabot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/lunabot

USER lunabot
RUN mkdir -p /home/lunabot/lunabot_ws
WORKDIR /home/lunabot/lunabot_ws

RUN echo 'unset GTK_PATH' >> ~/.bashrc \
    && echo 'source /opt/ros/lyrical/setup.bash' >> ~/.bashrc \
    && echo '[ -f ~/lunabot_ws/install/setup.bash ] && source ~/lunabot_ws/install/setup.bash' >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
