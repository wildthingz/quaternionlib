#!/usr/bin/bash
# to compile and run:                       ./run.sh -c
# to compile only:                          ./run.sh -co
# continue to run without compiling:        ./run.sh

PROJECT_NAME="quaternion"
BUILD_TYPE="release"
# BUILD_TYPE="debug"

## if -c is provided change the project in cmakelists to $PROJECT_NAME
[ "$1" == "-c" ] || [ "$1" == "-co" ] &&
sed -i "s/\(set[(]PROJECT_NAME \)\(.*\)\([)]\)/\1 $PROJECT_NAME\3/g" CMakeLists.txt


## if -c is provided delete the old build folder
[ "$1" == "-c" ] || [ "$1" == "-co" ] &&
[ -d "build" ] && rm -rf build 

## if -c is provided make build directory, and compile the code
[ "$1" == "-c" ] || [ "$1" == "-co" ] &&
mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE .. && make -j 4 && cd ..

[ "$1" != "-co" ] &&
./build/$PROJECT_NAME
