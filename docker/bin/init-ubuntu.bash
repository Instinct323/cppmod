#!/bin/bash

if [ $(id -u) -eq 0 ]; then
  apt update
  apt upgrade -y
  # utils
  apt install -y curl git sudo tree unzip wget
  apt install -y build-essential cmake gdb
  # pip
  apt install -y python3-pip
  pip config set global.timeout 6000
  pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
  pip config set global.trusted-host pypi.tuna.tsinghua.edu.cn
  pip install --upgrade pip
  # for Desktop
  apt install -y kazam
  xhost +local:

else
  echo "error: permission denied."
fi
