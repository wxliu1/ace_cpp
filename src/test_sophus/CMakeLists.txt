

cmake_minimum_required(VERSION 3.13)

project(test1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

# SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
# # set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g") # cmake default: "-O2 -g -DNDEBUG"
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(Sophus REQUIRED)
set(Eigen3_DIR "/root/SETUP/eigen-3.4.0_install/share/eigen3/cmake")
find_package(Eigen3 REQUIRED)
# set(Ceres_DIR "/root/SETUP/ceres210_install/lib/cmake/Ceres")
# find_package(Ceres REQUIRED)
find_package(fmt REQUIRED)
message(STATUS "Found {fmt} ${fmt_VERSION} in: ${fmt_DIR}")

# include_directories(
#   /root/SETUP/gperftools_install/include
# )

# link_directories(/root/SETUP/gperftools_install/lib)


add_executable(test1
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test1.cpp
)

target_link_libraries(test1 Eigen3::Eigen fmt::fmt)
# target_link_libraries(test1 -lpthread)


