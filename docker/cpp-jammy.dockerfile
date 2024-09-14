# docker build -f cpp-jammy.dockerfile -t instinct323/cpp-jammy:empty .

FROM ubuntu:22.04

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
RUN apt install -y openssh-server && \
    apt clean && \
    mkdir /var/run/sshd && \
    sed -ri 's/^PermitRootLogin\s+.*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -ri 's/UsePAM yes/#UsePAM yes/g' /etc/ssh/sshd_config

# C++ toolchain, cmake 3.22.1
RUN apt install -y build-essential cmake gdb && \
    apt clean

ARG BIN=/usr/local/bin
COPY bin/ros.key /usr/share/keyrings/ros-archive-keyring.gpg

CMD /usr/sbin/sshd -D
