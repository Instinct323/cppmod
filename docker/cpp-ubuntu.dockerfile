# docker build -f cpp-ubuntu.dockerfile -t cpp .
# docker run -p 22:22 cpp
# docker exec -it <ctn> bash

FROM ubuntu:22.04

ARG USER=tongzj
ARG PASSWD='20010323'
ARG EMAIL='1400721986@qq.com'

# windows: VcXsrv
ENV DISPLAY='host.docker.internal:0'
RUN useradd -m $USER && \
    echo $USER:$PASSWD | chpasswd

# apt
RUN apt update && \
    apt install -y tree unzip wget

# Git
RUN apt install -y git && \
    git config --global user.name $USER && \
    git config --global user.email $EMAIL

# OpenSSH
RUN DEBIAN_FRONTEND=noninteractive apt install -y openssh-server && \
    mkdir /var/run/sshd && \
    sed -ri 's/^PermitRootLogin\s+.*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -ri 's/UsePAM yes/#UsePAM yes/g' /etc/ssh/sshd_config

# C++ toolchain
ARG BIN=/usr/local/bin
COPY cpp-bin/*.bash $BIN/
COPY cpp-bin/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
RUN chmod +x $BIN/*.bash && \
    apt install -y build-essential cmake gdb

WORKDIR /home/$USER
CMD /usr/sbin/sshd -D
