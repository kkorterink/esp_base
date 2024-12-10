#!/bin/bash

rm -rf build
mkdir build
cd build
cmake ../ -G Ninja
cd ..
ninja -C build app
