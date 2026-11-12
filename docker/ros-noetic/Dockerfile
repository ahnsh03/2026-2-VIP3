# VIP3 parking runtime/dev tools. Build from the REPOSITORY ROOT.
# Image build includes workspace validation; it never starts a service container.
FROM ros:noetic-ros-core-focal@sha256:fd6279f04e51fa61201a791da9a156129941887b95b614ae0cec7865ef49de4f AS runtime-base

ARG UBUNTU_MIRROR=https://mirror.kakao.com/ubuntu

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=Asia/Seoul \
    QT_X11_NO_MITSHM=1 XDG_RUNTIME_DIR=/tmp/vip3-xdg-runtime \
    XCURSOR_PATH=/usr/share/icons XCURSOR_THEME=DMZ-White

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# No desktop-full/Gazebo, CUDA or ML checkpoints in the default runtime.
# Apt OpenCV and NumPy keep cv_bridge's system ABI consistent.
RUN printf 'Acquire::Retries "5";\nAcquire::http::Pipeline-Depth "0";\n' \
      > /etc/apt/apt.conf.d/99docker-apt-fix && \
    sed -i "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g; s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git ca-certificates iproute2 procps \
      python3-numpy python3-scipy python3-opencv python3-yaml \
      python3-nose python3-psutil python3-ujson python3-pil \
      ros-noetic-catkin ros-noetic-message-generation \
      ros-noetic-rosbridge-server ros-noetic-rosbag \
      ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
      ros-noetic-cv-bridge ros-noetic-compressed-image-transport \
      ros-noetic-rqt-image-view ros-noetic-rviz \
      ros-noetic-nav-msgs ros-noetic-std-srvs ros-noetic-visualization-msgs \
      dmz-cursor-theme && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/* && \
    ln -snf /usr/share/zoneinfo/Asia/Seoul /etc/localtime && \
    echo Asia/Seoul > /etc/timezone

COPY docker/ros-noetic/vip3-ros-gui /usr/local/bin/vip3-ros-gui
RUN chmod 0755 /usr/local/bin/vip3-ros-gui && \
    install -d -m 0700 /tmp/vip3-xdg-runtime && \
    git config --system --add safe.directory /root/ws && \
    git config --system --add safe.directory /root/ws/src/morai_msgs && \
    printf '%s\n' \
      'source /opt/ros/noetic/setup.bash' \
      '[ ! -f /root/ws/devel/setup.bash ] || source /root/ws/devel/setup.bash' \
      '[ ! -f /root/ws/scripts/vip3_env.sh ] || source /root/ws/scripts/vip3_env.sh' \
      >> /root/.bashrc
WORKDIR /root/ws
CMD ["bash"]

# Validation runs ONLY during docker build. No docker run/compose up is needed.
FROM runtime-base AS validation
WORKDIR /opt/vip3-check
COPY src ./src
COPY config ./config
COPY scripts ./scripts
COPY tools ./tools
COPY docker/ros-noetic/validate_image.sh /tmp/validate_image.sh
RUN bash /tmp/validate_image.sh 2>&1 | tee /tmp/vip3-image-validation.log

# Do not ship the build workspace, sources, generated messages or test caches.
# The user's checkout is mounted and built when they choose to create a container.
FROM runtime-base AS runtime
COPY --from=validation /tmp/vip3-image-validation.log /usr/share/vip3/image-validation.log
LABEL org.opencontainers.image.title="VIP3 parking ROS runtime" \
      org.opencontainers.image.description="ROS Noetic, rosbridge, four-fisheye SVM, rosbag and RViz; no ML training runtime"
WORKDIR /root/ws
