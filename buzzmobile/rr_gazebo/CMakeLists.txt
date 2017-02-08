cmake_minimum_required(VERSION 2.8.3)




add_executable(speedi_controller nodes/speedi_controller.cpp)
target_link_libraries(speedi_controller ${catkin_LIBRARIES})
add_dependencies(speedi_controller rr_platform_generate_messages_cpp)


install(PROGRAMS
        scripts/spawn_speedi_platform.sh
        scripts/controller_spawner.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
