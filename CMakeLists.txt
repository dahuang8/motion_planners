cmake_minimum_required(VERSION 2.9)
project(motionplan)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

file(GLOB PROJ_HEADER ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB PROJ_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)
set(PROJ_MAIN ${PROJECT_SOURCE_DIR}/test/main.cpp)
set(PROJ_TEST ${PROJECT_SOURCE_DIR}/test/unit_test.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/third_party/
        ${PYTHON_INCLUDE_DIRS})

enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(motionplan ${PROJ_MAIN} ${PROJ_SRC} ${PROJ_HEADER})
target_link_libraries(motionplan ${GTEST_BOTH_LIBRARIES} pthread ${PYTHON_LIBRARIES})

add_executable(motionplan_unit_test ${PROJ_TEST} ${PROJ_SRC} ${PROJ_HEADER})
target_link_libraries(motionplan_unit_test ${GTEST_BOTH_LIBRARIES} pthread ${PYTHON_LIBRARIES})
