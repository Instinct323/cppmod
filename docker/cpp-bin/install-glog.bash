#!/bin/bash

export TMP=/tmp/glog
git clone https://github.com/google/glog $TMP
cmake-install.bash $TMP
