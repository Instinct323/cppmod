# docker build -t instinct323/cpp-focal:empty -f cpp-ubuntu.dockerfile .

# CUDA-VERSION
ARG LSB_RELEASE=20.04
ARG CUDA_VERSION=12.2.2-cudnn8-devel

# FROM ubuntu:${LSB_RELEASE}
FROM nvidia/cuda:${CUDA_VERSION}-ubuntu${LSB_RELEASE}

ENV DEBIAN_FRONTEND=noninteractive

# apt
RUN apt update && \
    apt upgrade -y && \
    apt install -y curl git sudo tree unzip wget && \
    apt clean

# setup timezone
RUN echo 'Asia/Shanghai' > /etc/timezone && \
    apt install -q -y --no-install-recommends tzdata

# OpenSSH
ARG CFG=/etc/ssh/sshd_config
RUN apt update && \
    apt install -y openssh-server && \
    apt clean && \
    mkdir /var/run/sshd && \
    echo "PermitRootLogin yes" >> $CFG

# C++ toolchain, cmake
RUN apt install -y build-essential cmake gdb && \
    apt clean

# Python 3.9
RUN apt install -y python3.9 python3.9-dev && \
    apt clean && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1

# pip
RUN apt install -y python3-pip && \
    apt clean && \
    pip config set global.timeout 6000 && \
    pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip config set global.trusted-host pypi.tuna.tsinghua.edu.cn && \
    pip install --upgrade pip

# ROS
ARG BIN=/usr/local/bin
COPY bin/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
COPY bin/ros.* /tmp/

CMD /usr/sbin/sshd -D
