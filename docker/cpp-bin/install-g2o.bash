#!/bin/bash

export TMP=/tmp/g2o
apt install -y libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
git clone https://github.com/RainerKuemmerle/g2o $TMP
cmake-install.bash $TMP
