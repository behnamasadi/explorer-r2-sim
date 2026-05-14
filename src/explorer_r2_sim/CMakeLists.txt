cmake_minimum_required(VERSION 3.10)
project(explorer_r2_sim)

find_package(ament_cmake REQUIRED)

# Pure data package — no compiled targets. Just install the assets so
# `ros2 launch explorer_r2_sim cave.launch.py` can find them via
# get_package_share_directory().
install(DIRECTORY launch config rviz worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
