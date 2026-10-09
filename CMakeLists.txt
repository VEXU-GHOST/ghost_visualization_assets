cmake_minimum_required(VERSION 3.8)
project(ghost_visualization_assets)

find_package(ament_cmake REQUIRED)

# Install each available model directory into this package's ROS share path.
foreach(model baby_jerry main_robot field)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${model}")
    install(DIRECTORY ${model} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
