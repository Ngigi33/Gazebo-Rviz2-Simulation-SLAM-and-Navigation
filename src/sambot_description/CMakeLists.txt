cmake_minimum_required(VERSION 3.8)
project(sambot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add executable



ament_package()

install(
  DIRECTORY meshes urdf launch rviz worlds models
  DESTINATION share/${PROJECT_NAME}
)