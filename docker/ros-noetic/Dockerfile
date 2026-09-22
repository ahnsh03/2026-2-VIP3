# VIP3 표준 ROS1 개발 컨테이너 (Ubuntu 20.04 + ROS Noetic)
# MORAI 와의 전송은 rosbridge WebSocket(ws://127.0.0.1:9090) 하나뿐이다. UDP 는 쓰지 않는다.
FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

# Ubuntu mirror hash mismatch 완화 (Docker Desktop/WSL 간헐적)
RUN printf 'Acquire::Retries "5";\nAcquire::http::Pipeline-Depth "0";\n' \
    > /etc/apt/apt.conf.d/99docker-apt-fix

RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales && \
    ln -fs /usr/share/zoneinfo/Asia/Seoul /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8 ko_KR.UTF-8 && \
    update-locale LANG=en_US.UTF-8

# desktop-full 이 대부분을 이미 포함하지만, 카메라 4대 CompressedImage 디버깅이 이
# 프로젝트의 핵심 경로라 image_transport / compressed / rqt_image_view 를 명시 고정한다.
# (libpcap-dev 는 Velodyne UDP 패킷 캡처 전용이라 뺐다.)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    git curl wget nano vim tmux sudo bash-completion \
    build-essential cmake libyaml-cpp-dev libgeographic-dev \
    python3-pip python3-dev \
    python3-scipy python3-numpy \
    iputils-ping net-tools iproute2 mesa-utils \
    ros-noetic-rosbridge-server \
    ros-noetic-rosbridge-suite \
    ros-noetic-image-transport \
    ros-noetic-compressed-image-transport \
    ros-noetic-image-transport-plugins \
    ros-noetic-rqt-image-view \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-cv-bridge \
    dmz-cursor-theme xcursor-themes \
    && rm -rf /var/lib/apt/lists/*

COPY requirements-perception.txt requirements-hdmap.txt /tmp/
RUN pip3 install --no-cache-dir --upgrade "pip<24.1"

# 시스템 Python 3.8 용 CUDA PyTorch. TwinLiteNet+ 추론(camera_semantic_perception)이
# 이 인터프리터를 쓴다. RTX 30/40(sm_86/sm_89)까지 커버한다.
# RTX 50(sm_120)은 cu118 wheel 로 동작하지 않는다 — Dockerfile.rtx50 참고.
RUN pip3 install --no-cache-dir \
      torch==2.1.2 \
      torchvision==0.16.2 \
      --index-url https://download.pytorch.org/whl/cu118

# Ubuntu/ROS 가 distutils 로 설치한 패키지는 pip 가 제거할 수 없으므로 필요한 버전을
# /usr/local 에 overlay 한다.
# ujson: rosbridge_library/util/__init__.py 가 `try: import ujson as json` 으로 자동
#        선택한다. 카메라 4대 CompressedImage 를 base64 JSON 으로 나르는 VIP3 에서
#        가장 값싼 처리량 개선이다.
RUN pip3 install --no-cache-dir --ignore-installed \
      "PyYAML>=5.4" \
      "numpy==1.24.4" \
      "scipy==1.10.1" \
      "matplotlib==3.7.5" \
      "pillow==10.4.0" \
      "psutil>=5.8" \
      "ujson"

RUN pip3 install --no-cache-dir -r /tmp/requirements-perception.txt && \
    pip3 install --no-cache-dir -r /tmp/requirements-hdmap.txt && \
    pip3 install --no-cache-dir tqdm && \
    rm -f /tmp/requirements-*.txt && \
    python3 -c "from scipy.spatial import cKDTree; import cv2, numpy, h5py, pyproj, shapely, ujson, torch; print(torch.__version__, torch.cuda.is_available())"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/ws/devel/setup.bash ] && source /root/ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export LANG=en_US.UTF-8" >> /root/.bashrc && \
    echo "export LC_ALL=en_US.UTF-8" >> /root/.bashrc

# Qt 는 XDG_RUNTIME_DIR 이 존재하고 소유자가 현재 사용자이며 mode 0700 이기를 요구한다.
# 호스트 WSL 경로를 상속하면 컨테이너 안에 그 경로가 없어 rqt/RViz 가 죽는다.
# vip3-ros-gui 래퍼는 Qt geometry 도 매번 초기화해 창이 화면 밖으로 열리는 것을 막는다.
COPY vip3-ros-gui /usr/local/bin/vip3-ros-gui
RUN chmod 0755 /usr/local/bin/vip3-ros-gui && \
    install -d -m 0700 /tmp/vip3-xdg-runtime

WORKDIR /root/ws

ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/vip3-xdg-runtime
ENV XCURSOR_PATH=/usr/share/icons
ENV XCURSOR_THEME=DMZ-White

CMD ["bash"]
