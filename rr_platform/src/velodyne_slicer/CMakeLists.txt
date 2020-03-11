add_executable(velodyne_slicer velodyne_slicer.cpp)
target_link_libraries(velodyne_slicer ${catkin_LIBRARIES} rr_serial)
add_dependencies(velodyne_slicer ${catkin_EXPORTED_TARGETS})