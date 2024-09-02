#!/bin/bash

export TMP=/tmp/cppmod
export BIN=/usr/local/bin

git clone https://github.com/Instinct323/cppmod $TMP
cp $TMP/docker/$(lsb_release -cs)/*.bash $BIN/
chmod +x $BIN/*.bash
rm -rf $TMP
