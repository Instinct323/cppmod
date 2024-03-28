#!/bin/bash

export TMP=/tmp/fmt
git clone -b 10.2.1 https://github.com/fmtlib/fmt $TMP
cmake-install.bash $TMP
