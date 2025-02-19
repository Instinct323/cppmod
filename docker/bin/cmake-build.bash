#!/bin/bash
# cmake-build.bash <repo-path>

cd $1
mkdir build && cd build && cmake .. && make
