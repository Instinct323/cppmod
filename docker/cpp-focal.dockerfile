# docker build -f cpp-focal.dockerfile -t cpp-focal .
# docker run -p 22:22 -v D:\Workbench:/home/workbench cpp-focal
# docker exec -it <ctn> bash

FROM ubuntu:20.04

ARG USER=tongzj
ARG PASSWD='20010323'
ARG EMAIL='1400721986@qq.com'

# windows: VcXsrv
ENV DISPLAY='host.docker.internal:0'
ENV DEBIAN_FRONTEND=noninteractive

RUN useradd -m $USER && \
    echo $USER:$PASSWD | chpasswd && \
    echo root:$PASSWD | chpasswd && \
    usermod -aG sudo $USER

# apt
RUN apt update && \
    apt install -y tree unzip wget sudo python3-pip && \
    apt clean

# Git
RUN apt install -y git && \
    apt clean && \
    git config --global user.name $USER && \
    git config --global user.email $EMAIL

# OpenSSH
RUN apt install -y openssh-server && \
    apt clean && \
    mkdir /var/run/sshd && \
    sed -ri 's/^PermitRootLogin\s+.*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -ri 's/UsePAM yes/#UsePAM yes/g' /etc/ssh/sshd_config

# C++ toolchain, cmake 3.16.3
RUN apt install -y build-essential cmake gdb && \
    apt clean

ARG BIN=/usr/local/bin
COPY cpp-bin/*.bash $BIN/
COPY focal/*.bash $BIN/
COPY focal/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
COPY focal/ros.* /tmp/
RUN chmod +x $BIN/*.bash

WORKDIR /home/$USER
CMD /usr/sbin/sshd -D
