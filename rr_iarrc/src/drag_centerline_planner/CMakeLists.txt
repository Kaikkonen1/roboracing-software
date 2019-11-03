add_executable(drag_centerline_planner drag_centerline_planner.cpp)
target_link_libraries(drag_centerline_planner ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(drag_centerline_planner_contours drag_centerline_planner_contours.cpp)
target_link_libraries(drag_centerline_planner_contours ${catkin_LIBRARIES} ${OpenCV_LIBS})
