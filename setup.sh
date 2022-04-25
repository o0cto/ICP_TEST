#!/bin/bash

SRC_DIR=`pwd`
export LD_LIBRARY_PATH=${SRC_DIR}/third/lib:$LD_LIBRARY_PATH


echo ">>>> build ..."
mkdir build
cd build
cmake -DCMAKE_CXX_FLAGS=-fPIC ..
make

echo ">>>> test ..."
./testICP
