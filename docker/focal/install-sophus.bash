#!/bin/bash

# REQUIRE eigen, fmt
export TMP=/tmp/Sophus
git clone https://github.com/strasdat/Sophus $TMP
cmake-install.bash $TMP
