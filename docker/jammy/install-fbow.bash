#!/bin/bash

# REQUIRE opencv
export TMP=/tmp/fbow
git clone https://github.com/rmsalinas/fbow $TMP
cmake-install.bash $TMP
