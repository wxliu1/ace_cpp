cmake_minimum_required(VERSION 3.5)
project(loop_test)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic) # -Wall : print warning

# set(CMAKE_BUILD_TYPE Release)

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

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV)
find_package(Ceres REQUIRED)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
find_package(yaml-cpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
find_package(loop_action REQUIRED)

include_directories(
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
)

file(GLOB LOOP_SRC
  src/keyframe.cpp
  src/utility/utility.cpp
  src/ThirdParty/DBoW/BowVector.cpp
  src/ThirdParty/DBoW/FBrief.cpp
  src/ThirdParty/DBoW/FeatureVector.cpp
  src/ThirdParty/DBoW/QueryResults.cpp
  src/ThirdParty/DBoW/ScoringObject.cpp
  src/ThirdParty/DUtils/Random.cpp
  src/ThirdParty/DUtils/Timestamp.cpp
  src/ThirdParty/DVision/BRIEF.cpp
  src/ThirdParty/VocabularyBinary.cpp
)

add_executable(${PROJECT_NAME}_main
  ${LOOP_SRC}
  src/loop_main.cpp
)
target_include_directories(${PROJECT_NAME}_main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME}_main
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  #OpenCL
  yaml-cpp)
