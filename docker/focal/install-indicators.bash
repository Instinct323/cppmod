#!/bin/bash

export TMP=/tmp/indicators
git clone https://github.com/p-ranav/indicators $TMP
cp -r $TMP/include /usr/local
rm -rf $TMP
