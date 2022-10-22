#!/bin/bash

#set -eu

# ./build_ikfast.sh ik_fast_utra6_550_robot
g++ ikfast_test_fk.cpp ${1}.cpp -o ${1} -llapack -std=c++11
