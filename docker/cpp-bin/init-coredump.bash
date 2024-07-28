#!/bin/bash

DUMP_PATH=`pwd`/coredump

if [ "$(id -u)" != "0" ]; then
  echo "Please run as root user."
  exit 1
fi

# mkdir
mkdir -p $DUMP_PATH
chmod 777 $DUMP_PATH

# default: 0
echo 2 > /proc/sys/fs/suid_dumpable
# default: /mnt/wslg/dumps/core.%e
echo "$DUMP_PATH/core.%e" > /proc/sys/kernel/core_pattern
