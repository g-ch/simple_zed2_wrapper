cmake_minimum_required(VERSION 3.5)
PROJECT(simple_zed2_wrapper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE "Release")

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
# SET(OpenGL_GL_PREFERENCE GLVND)
# find_package(OpenGL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
    tf2
    tf2_ros
	geometry_msgs
	std_msgs
    shape_msgs
	visualization_msgs
	message_generation
)


add_message_files(
    FILES
    "Object.msg"
    "ObjectsStamped.msg"
    "Keypoint2Di.msg"
    "Keypoint2Df.msg"
    "Keypoint3D.msg"
    "BoundingBox2Di.msg"
    "BoundingBox2Df.msg"
    "BoundingBox3D.msg"
    "Skeleton2D.msg"
    "Skeleton3D.msg"
    "DepthInfoStamped.msg"
    "PlaneStamped.msg"
    "PosTrackStatus.msg"
    "GnssFusionStatus.msg"
    "Heartbeat.msg"
    "MagHeadingStatus.msg"
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  shape_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

IF (CMAKE_SYSTEM_PROCESSOR MATCHES aarch64)
    add_definitions(-DJETSON_STYLE)
ENDIF()


ADD_EXECUTABLE(zed2_node src/zed2_node.cpp)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

# add_subdirectory(${PROJECT_SOURCE_DIR}/include/backward-cpp)


target_link_libraries(zed2_node
                        # PUBLIC Backward::Interface 
                        ${ZED_LIBS}
                        ${OPENGL_LIBRARIES}
                        ${GLUT_LIBRARIES}
                        ${GLEW_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        ${catkin_LIBRARIES})

