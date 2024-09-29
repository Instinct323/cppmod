# docker build -f cpp-focal.dockerfile -t instinct323/cpp-focal:empty .

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

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

# C++ toolchain, cmake 3.16.3
RUN apt install -y build-essential cmake gdb && \
    apt clean

ARG BIN=/usr/local/bin
COPY bin/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
COPY bin/ros.* /tmp/

CMD /usr/sbin/sshd -D
