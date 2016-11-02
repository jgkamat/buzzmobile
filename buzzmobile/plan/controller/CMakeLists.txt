add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(controller ${catkin_EXPORTED_TARGETS}) 
