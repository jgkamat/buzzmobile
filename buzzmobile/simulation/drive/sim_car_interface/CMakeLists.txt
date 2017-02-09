add_executable(sim_car_interface src/sim_car_interface.cpp)
target_link_libraries(sim_car_interface ${catkin_LIBRARIES})
add_dependencies(sim_car_interface ${catkin_EXPORTED_TARGETS}) 
