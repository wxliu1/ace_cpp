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


message(STATUS "CMAKE_CXX_FLAGS_DEBUG=${CMAKE_CXX_FLAGS_DEBUG}") 

# for gdb debug.
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# the end.

# SET(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -ggdb")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-pg") # cmake default: "-O2 -g -DNDEBUG"
message(STATUS "CMAKE_CXX_FLAGS_RELWITHDEBINFO=${CMAKE_CXX_FLAGS_RELWITHDEBINFO}") 
# message(STATUS "CMAKE_CXX_FLAGS_DEBUG=${CMAKE_CXX_FLAGS_DEBUG}") 
message(STATUS "CMAKE_CXX_FLAGS_DEBUG=" ${CMAKE_CXX_FLAGS_DEBUG}) 
message(STATUS "ENV{CXXFLAGS}=$ENV{CXXFLAGS}")

find_package(TBB REQUIRED)
# find_package(Eigen3 REQUIRED)
set(Eigen3_DIR "/root/SETUP/eigen-3.4.0_install/share/eigen3/cmake")
# set(Ceres_DIR "/root/SETUP/ceres210_install/lib/cmake/Ceres")
find_package(Eigen3 REQUIRED)

include_directories(
  /root/SETUP/gperftools_install/include
)

link_directories(/root/SETUP/gperftools_install/lib)

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

add_executable(test_container
  src/test_container.cpp
)

add_executable(test_quaternion
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test_quaternion.cpp
)

target_link_libraries(test_quaternion Eigen3::Eigen)


add_executable(test_tbb_concurrent_bounded_queue
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test_tbb_concurrent_bounded_queue.cpp
)

target_link_libraries(test_tbb_concurrent_bounded_queue TBB::tbb)

add_executable(test_gperftools
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test_gperftools.cpp
)

target_link_libraries(test_gperftools -lprofiler)


# 2024-08-29
option(DEBUG_ASAN "AddressSanitizer, Debug for memory leak" OFF) # ON or OFF
option(DEBUG_TSAN "ThreadSanitizer, Debug for data race" OFF) # ON or OFF
############# Compile flags ###############
if (DEBUG_ASAN)
    SET(CMAKE_BUILD_TYPE Debug)
    # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    # set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} -fsanitize=address")

    SET(CMAKE_CXX_FLAGS_DEBUG "-g -fsanitize=address -fno-omit-frame-pointer")
endif ()

if (DEBUG_TSAN)
    SET(CMAKE_BUILD_TYPE Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread -fno-omit-frame-pointer")
    set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} -fsanitize=thread")
endif ()
# the end

add_executable(test_tcmalloc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/test_tcmalloc.cpp
)

target_link_libraries(test_tcmalloc -lprofiler -ltcmalloc -lpthread)

# add_executable(epoll_server_test src/epoll_server_test.cpp)

# target_link_libraries(ace_cpp ${OpenCV_LIBS} yaml-cpp jsoncpp mysqlclient tinyxml) # '-lpthread'
# target_include_directories(ace_cpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)