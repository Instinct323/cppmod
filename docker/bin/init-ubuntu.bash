#!/bin/bash

if [ $(id -u) -eq 0 ]; then
  apt update
  apt upgrade -y
  # utils
  apt install -y curl git python3-pip sudo tree unzip wget
  apt install -y build-essential cmake gdb
  # for Desktop
  apt install -y kazam

else
  echo "error: permission denied."
fi
