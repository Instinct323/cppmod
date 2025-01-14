# docker build -f cpp-ubuntu.dockerfile -t instinct323/cpp-focal:empty .

# CUDA-VERSION: 12.6.3-cudnn-devel
ARG LSB_RELEASE=20.04
ARG CUDA_VERSION=12.6.3-cudnn-devel
ENV DEBIAN_FRONTEND=noninteractive

# FROM ubuntu:${LSB_RELEASE}
FROM nvidia/cuda:${CUDA_VERSION}-ubuntu${LSB_RELEASE}

# apt
RUN apt update && \
    apt install -y curl git python3-pip sudo tree unzip wget && \
    apt clean

# setup timezone
RUN echo 'Asia/Shanghai' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt install -q -y --no-install-recommends tzdata

# OpenSSH
RUN apt update && \
    apt install -y openssh-server && \
    apt clean && \
    mkdir /var/run/sshd && \
    echo "PermitRootLogin yes" >> /etc/ssh/sshd_config

# C++ toolchain, cmake
RUN apt install -y build-essential cmake gdb && \
    apt clean

ARG BIN=/usr/local/bin
COPY bin/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
COPY bin/ros.* /tmp/

CMD /usr/sbin/sshd -D
