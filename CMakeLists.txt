cmake_minimum_required(VERSION 3.8)
project(rrt_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)

# set(msg_files PointArray.msg) # Probar. Si no, incluir nombre y extension abajo, junto a std_msgs

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PointArray.msg
  action/ExploreNextPoint.action
  DEPENDENCIES std_msgs geometry_msgs 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include
                    ${OpenCV_INCLUDE_DIRS}
)

add_executable(local_rrt_detector src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)

add_executable(global_rrt_detector src/global_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)

ament_target_dependencies(local_rrt_detector
                          # global_rrt_detector
                          rclcpp
                          std_msgs
                          nav_msgs
                          visualization_msgs
                          tf2_ros)
ament_target_dependencies(# local_rrt_detector
                          global_rrt_detector
                          rclcpp
                          std_msgs
                          nav_msgs
                          visualization_msgs
                          tf2_ros)


target_link_libraries(local_rrt_detector ${OpenCV_LIBRARIES})

target_link_libraries(global_rrt_detector ${OpenCV_LIBRARIES})

install(DIRECTORY include/
   DESTINATION include)

install(TARGETS
   local_rrt_detector
   global_rrt_detector
 
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION lib/${PROJECT_NAME}
 )

install(PROGRAMS
  scripts/assigner.py
  scripts/filter.py
  scripts/frontier_opencv_detector.py
  scripts/functions.py
  scripts/getfrontier.py
  scripts/robot.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    launch
    config
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies()

ament_package()


# #cmake_minimum_required(VERSION 2.8.3)
# cmake_minimum_required(VERSION 3.5)
# project(talker)
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
# #find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# #catkin_package()
# #include_directories(${catkin_INCLUDE_DIRS})
# include_directories(include)
# add_executable(talker talker.cpp)
# #target_link_libraries(talker ${catkin_LIBRARIES})
# ament_target_dependencies(talker
#   rclcpp
#   std_msgs)
# #install(TARGETS talker
# #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(TARGETS talker
#   DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY include/
#   DESTINATION include)
# ament_export_include_directories(include)
# ament_export_dependencies(std_msgs)
# ament_package()
