add_executable(car_interface src/main.cpp src/arduino.cpp)
target_link_libraries(car_interface ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(car_interface ${catkin_EXPORTED_TARGETS}) 
