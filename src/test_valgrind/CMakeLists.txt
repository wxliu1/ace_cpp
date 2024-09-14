
cmake_minimum_required(VERSION 3.13)

project(test_valgrind)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g") # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")

# SET(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

add_executable(test_valgrind
  src/test_valgrind.cpp
)

add_executable(test_valgrind2
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test_valgrind2.cpp
)