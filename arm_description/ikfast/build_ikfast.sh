#!/bin/bash

#set -eu

# ./build_ikfast.sh ik_fast_utra6_550_robot
g++ ikfast_test_fk.cpp ${1}.cpp -o ${1}_fk -llapack -std=c++11
g++ ikfast_test_ik.cpp ${1}.cpp -o ${1}_ik -llapack -std=c++11

