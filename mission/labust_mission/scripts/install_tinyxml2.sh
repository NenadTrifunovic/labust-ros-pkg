#!/bin/bash
git clone https://github.com/leethomason/tinyxml2
cd tinyxml2
rm -rf build
mkdir -p build
cd build
cmake ..
make
sudo make install
cd ../../
rm -rf tinyxml2
