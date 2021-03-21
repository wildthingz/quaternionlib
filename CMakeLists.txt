cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME  quat2eul)

project(${PROJECT_NAME})

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} include/${PROJECT_NAME})

link_directories(src)
add_subdirectory(src)

add_executable(${PROJECT_NAME} src/main.cpp) 
target_link_libraries(${PROJECT_NAME} PRIVATE quat2eullib)
