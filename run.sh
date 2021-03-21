#!/usr/bin/bash
# to compile and run:                       ./run.sh -c
# to compile only:                          ./run.sh -co
# continue to run without compiling:        ./run.sh
# run new without compiling:                ./run.sh -n

PROJECT_NAME="quat2eul"

## if -c is provided change the project in cmakelists to $PROJECT_NAME
[ "$1" == "-c" ] || [ "$1" == "-co" ] &&
sed -i "s/\(set[(]PROJECT_NAME \)\(.*\)\([)]\)/\1 $PROJECT_NAME\3/g" CMakeLists.txt


## if -c is provided delete the old build folder
[ "$1" == "-c" ] || [ "$1" == "-co" ] &&
[ -d "build" ] && rm -rf build 

## if -c is provided make build directory, and compile the code
[ "$1" == "-c" ] || [ "$1" == "-co" ] &&
mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=release .. && make -j 4 && cd ..

## delete results folder if exists
[ "$1" == "-c" ] || [ "$1" == "-co" ] || [ "$1" == "-n" ] && 
[ -d "results" ] && rm -rf results

## make results folder and run the executable
[ "$1" == "-c" ] || [ "$1" == "-co" ] || [ "$1" == "-n" ] &&
mkdir results

cd results

## move results to a different folder
# [ -d "results" ] && mv results results.old

[ "$1" != "-co" ] &&
../build/$PROJECT_NAME
