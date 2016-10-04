project(buzzmobile)
set(CMAKE_CXX_FLAGS "-std=c++11 -g ${CMAKE_CXX_FLAGS}")
list(APPEND CMAKE_MODULE_PATH $(CMAKE_CURRENT_SOURCE_DIR)/cmake)
find_package(catkin REQUIRED COMPONENTS
              roscpp
              std_msgs
	      sensor_msgs
              cv_bridge
	      image_transport
              message_generation
              )
find_package(OpenCV REQUIRED)
catkin_package( INCLUDE_DIRS
                LIBRARIES buzzmobile
                CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime cv_bridge
                DEPENDS system_lib
)
include_directories(
                    include
                    ${catkin_INCLUDE_DIRS}
                    ${OPENCV_INCLUDE_DIRS}
)
add_subdirectory(process/projector)
add_subdirectory(process/lane_detector)
add_subdirectory(sense/image_const)

add_message_files(
  FILES
  CarPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)
