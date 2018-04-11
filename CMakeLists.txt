cmake_minimum_required(VERSION 3.10)
project(rrt)

set(CMAKE_CXX_STANDARD 11)

file(GLOB PROJ_HEADER ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB PROJ_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include/)

enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(rrt ${PROJ_SRC} ${PROJ_HEADER})
target_link_libraries(rrt ${GTEST_BOTH_LIBRARIES} pthread)
