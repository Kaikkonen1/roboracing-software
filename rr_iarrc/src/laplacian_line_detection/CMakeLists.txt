find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_library(lane_detector laplacian_line_detector.cpp)
add_dependencies(lane_detector ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(lane_detector ${catkin_LIBRARIES})
target_link_libraries(lane_detector ${OpenCV_LIBS} )
add_dependencies(lane_detector rr_platform_generate_messages_cpp)


