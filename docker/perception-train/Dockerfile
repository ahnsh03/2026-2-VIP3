# VIP3 인지 모델 학습 컨테이너 (TwinLiteNet+ 전용).
# 베이스: Ubuntu 20.04 + CUDA 11.8 — upstream TwinLiteNet+ 가 검증된 조합.
# ROS 는 넣지 않는다. 학습 경로(scripts/train_twinlite*.py -> twinlite_morai ->
# model.model)는 torch/numpy/cv2/matplotlib 만 import 한다.
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip python3-dev python3-venv \
    git wget curl nano vim \
    build-essential cmake \
    libgl1-mesa-glx libglib2.0-0 libsm6 libxext6 libxrender-dev \
    software-properties-common lsb-release gnupg2 \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir --ignore-installed PyYAML==6.0.1

COPY requirements-train.txt /tmp/
RUN python3 -m pip install --no-cache-dir --retries 10 --timeout 120 \
    -r /tmp/requirements-train.txt && \
    rm /tmp/requirements-train.txt

RUN python3 -m pip install --no-cache-dir --retries 20 --timeout 300 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    --index-url https://download.pytorch.org/whl/cu118

WORKDIR /root/ws

CMD ["/bin/bash"]
