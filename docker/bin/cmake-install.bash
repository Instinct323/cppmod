#!/bin/bash
# cmake-install.bash <repo-path>

if [ $(id -u) -eq 0 ]; then
  cmake-build.bash $1

  cd $1/build
  make install

else
  echo "error: permission denied."
fi
