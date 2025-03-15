#!/bin/bash

if [ $(id -u) -eq 0 ]; then
  export TMP=/tmp/opencv
  apt install -y libgtk2.0-dev libjpeg-dev libopenexr-dev libtbb-dev

  git clone https://github.com/opencv/opencv $TMP
  git clone https://github.com/opencv/opencv_contrib $TMP_contrib

  cd $TMP
  mkdir build && cd build
  cmake -DOPENCV_EXTRA_MODULES_PATH=$TMP_contrib/modules ..
  make
  make install

  # cmake-install.bash $TMP
  # ln -s /usr/local/include/opencv4/opencv2 /usr/local/include
else
  echo "error: permission denied."
fi
