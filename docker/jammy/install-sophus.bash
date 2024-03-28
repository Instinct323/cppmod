#!/bin/bash

# after eigen, fmt
export TMP=/tmp/Sophus
git clone -b 1.22.10 https://github.com/strasdat/Sophus $TMP
cmake-install.bash $TMP
