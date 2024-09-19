#!/bin/bash

export TMP=/tmp/cppmod
export BIN=/usr/local/bin

git clone https://github.com/Instinct323/cppmod $TMP
cp $TMP/docker/bin/* $BIN/
cp $TMP/docker/$(lsb_release -cs)/*.bash $BIN/
chmod +x $BIN/*.bash
rm -rf $TMP

# sed -i "s/\r$//" bin/*
echo -e "\nls $BIN"
ls $BIN
