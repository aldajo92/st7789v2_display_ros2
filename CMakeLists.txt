cmake_minimum_required(VERSION 3.8)
project(st7789v2_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Build bcm2835 library
add_library(bcm2835 STATIC
  src/bcm2835.c
)
target_include_directories(bcm2835 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/st7789v2_display>
  $<INSTALL_INTERFACE:include/st7789v2_display>
)

# Install bcm2835 library
install(TARGETS bcm2835
  DESTINATION lib/${PROJECT_NAME}
)

# Build Waveshare LCD library
add_library(waveshare_lcd STATIC
  src/waveshare/DEV_Config.c
  src/waveshare/LCD_1in69.c
  src/waveshare/LCD_1in69_fast.c
  src/waveshare/GUI_Paint.c
  src/waveshare/font8.c
  src/waveshare/font12.c
  src/waveshare/font16.c
  src/waveshare/font20.c
  src/waveshare/font24.c
)
target_include_directories(waveshare_lcd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/st7789v2_display>
)
target_compile_definitions(waveshare_lcd PUBLIC USE_BCM2835_LIB)
target_link_libraries(waveshare_lcd bcm2835)

# Build display node executable
add_executable(display_node
  src/display_node_waveshare.cpp
)
target_include_directories(display_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(display_node
  rclcpp
  std_msgs
)
target_link_libraries(display_node waveshare_lcd bcm2835)

# Build image display node executable
add_executable(image_display_node
  src/image_display_node.cpp
)
target_include_directories(image_display_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(image_display_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(image_display_node 
  waveshare_lcd 
  bcm2835
  ${OpenCV_LIBS}
)

# Install targets
install(TARGETS 
  display_node
  image_display_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
