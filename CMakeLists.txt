#cmake_minimum_required(VERSION 3.20)
#project(k4a_read_mkv LANGUAGES CXX)



#set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_CXX_FLAGS "-Ofast")

#find_package(k4a REQUIRED)
#find_package(k4arecord REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(apriltag REQUIRED)

#message(STATUS "OpenCV : ${OpenCV_LIBRARIES}")


#add_executable(k4a_read_mkv main.cpp)

#target_include_directories(k4a_read_mkv PUBLIC include ${OpenCV_INCLUDE_DIRS})

#target_link_libraries(k4a_read_mkv PUBLIC
#   k4a # Kinect SDK
#   k4arecord 
#   apriltag::apriltag
    #opencv_core opencv_imgproc opencv_calib3d opencv_aruco 
    #   ${OpenCV_LIBRARIES}
    #stdc++fs # filesystem
   
    #)


cmake_minimum_required(VERSION 3.20)
project(k4a_read_mkv LANGUAGES CXX)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-Ofast")

# 1. Search for OpenMP
find_package(OpenMP REQUIRED)

find_package(k4a REQUIRED)
find_package(k4arecord REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)


message(STATUS "OpenCV : ${OpenCV_LIBRARIES}")

file(GLOB SOURCES "*.cpp")

add_executable(k4a_read_mkv ${SOURCES})


target_include_directories(k4a_read_mkv PUBLIC include ${OpenCV_INCLUDE_DIRS})

# 2. Add the OpenMP flags to the compiler flags
if(OpenMP_CXX_FOUND)
    target_link_libraries(k4a_read_mkv PUBLIC OpenMP::OpenMP_CXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



# Link against libraries
target_link_libraries(k4a_read_mkv PUBLIC
    k4a # Kinect SDK
    k4arecord 
    apriltag::apriltag
    #opencv_core opencv_imgproc opencv_calib3d opencv_aruco 
    ${OpenCV_LIBRARIES}
    stdc++fs # filesystem
)
