cmake_minimum_required(VERSION 3.13)

project(ace_cpp)

set (PackageName "ace_cpp" )

# set(CMAKE_CXX_FLAGS "-std=c++11")

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# for gdb debug.
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# the end.

# SET(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(test_list #src/main.cpp
  #${CMAKE_CURRENT_SOURCE_DIR}/src/wx_utility/compile_date_time.cpp
  # ${CMAKE_CURRENT_SOURCE_DIR}/src/wx_yaml_io.cpp
  src/test_list.cpp
)

add_executable(test_parallel_reduce
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test_parallel_reduce.cpp
)

target_link_libraries(test_parallel_reduce TBB::tbb)


add_executable(test_eigen
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test_eigen.cpp
)

target_link_libraries(test_eigen Eigen3::Eigen)


add_executable(test_normal_distribution
  src/test_normal_distribution.cpp
)

add_executable(test_cast
  src/test_cast.cpp
)

# add_executable(epoll_server_test src/epoll_server_test.cpp)

# target_link_libraries(ace_cpp ${OpenCV_LIBS} yaml-cpp jsoncpp mysqlclient tinyxml) # '-lpthread'
# target_include_directories(ace_cpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)