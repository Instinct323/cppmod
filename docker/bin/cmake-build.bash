#!/bin/bash
# cmake-build.bash <repo-path>

if [ $(id -u) -eq 0 ]; then
  mkdir $1/cmake-build
  cd $1/cmake-build

  cmake ..
  make

else
  echo "error: permission denied."
fi
