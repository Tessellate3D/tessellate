cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

project(tessellate)

set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(PROJECT_BINARY_DIR "${PROJECT_SOURCE_DIR}/bin")
set(LIBREALSENSE_INSTALLATION_DIR "/home/nvidia/librealsense")

#find_package(realsense2 REQUIRED PATHS ${LIBREALSENSE_INSTALLATION_DIR} NO_DEFAULT_PATH)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(glfw3 REQUIRED)
find_package(ZeroMQ REQUIRED)
find_package(cppzmq REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)


include_directories(${PROJECT_INCLUDE_DIR})

include_directories(${Eigen3_INCLUDE_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${Boost_INCLUDE_DIRS})

include_directories(${ZeroMQ_INCLUDE_DIRS} ${cppzmq_INCLUDE_DIR})

include_directories(${OPENGL_INCLUDE_DIR})

include_directories(${GLEW_INCLUDE_DIRS})
link_libraries(${GLEW_LIBRARIES})

include_directories(${GLUT_INCLUDE_DIRS})

include_directories(
  "/home/nvidia/librealsense/examples"
  "${LIBREALSENSE_INSTALLATION_DIR}/include"
)
link_directories("/usr/local/lib")
set(realsense_libs realsense2)

link_directories(${realsense_LIBRARY})
include_directories(${realsense_INCLUDE_DIR})

add_executable(pointcloud_sr300 ${PROJECT_SOURCE_DIR}/src/pointclouds.cpp)
target_link_libraries(pointcloud_sr300 
  ${PCL_LIBRARIES} 
  ${realsense_libs}
  ${OpenCV_LIBS} 
  ${Boost_INCLUDE_DIRS}
  ${OPENGL_LIBRARIES}
  glfw
  ${GLUT_LIBRARY}
  ${cppzmq_LIBRARY}
)
