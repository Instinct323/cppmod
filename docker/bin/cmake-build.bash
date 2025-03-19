#!/bin/bash
# cmake-build.bash <repo-path>

cd $1
mkdir -p build && cd build && cmake .. && make
