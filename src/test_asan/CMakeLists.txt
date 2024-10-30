cmake_minimum_required(VERSION 3.13)

project(test_asan)

# set (PackageName "test_asan" )

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
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# the end.

# SET(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

option(DEBUG_ASAN "AddressSanitizer, Debug for memory leak" ON) # ON or OFF
option(DEBUG_TSAN "ThreadSanitizer, Debug for data race" OFF) # ON or OFF
# 2024-08-29
############# Compile flags ###############
if (DEBUG_ASAN)
    SET(CMAKE_BUILD_TYPE Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} -fsanitize=address")
endif ()

if (DEBUG_TSAN)
    SET(CMAKE_BUILD_TYPE Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread -fno-omit-frame-pointer")
    set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} -fsanitize=thread")
endif ()
# the end


add_executable(test_asan
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test_asan.cpp
)

#target_link_libraries(test_asan -lasan)

# add_executable(epoll_server_test src/epoll_server_test.cpp)

# target_link_libraries(ace_cpp ${OpenCV_LIBS} yaml-cpp jsoncpp mysqlclient tinyxml) # '-lpthread'
# target_include_directories(ace_cpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
