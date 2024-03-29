#!/bin/bash

# REQUIRE eigen
export TMP=/tmp/g2o
apt install -y libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
git clone -b 20230223_git https://github.com/RainerKuemmerle/g2o $TMP

export CMAKE_MODULE=/usr/local/share/g2o/cmake
mkdir -p $CMAKE_MODULE
cp -r $TMP/cmake_modules/* $CMAKE_MODULE
cmake-install.bash $TMP
