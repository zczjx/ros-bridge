cmake_minimum_required(VERSION 3.5)
project(carla_video_encode)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# find ffmpeg libs
set(ffmpeg_libs_DIR /opt/ffmpeg/lib)
set(ffmpeg_headers_DIR /opt/ffmpeg/include)
include_directories(${ffmpeg_headers_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_AVFORMAT REQUIRED libavformat>=59)
pkg_check_modules(PC_AVFILTER REQUIRED libavfilter>=8)
pkg_check_modules(PC_AVCODEC REQUIRED libavcodec>=59)
pkg_check_modules(PC_AVUTIL REQUIRED libavutil>=57)
pkg_check_modules(PC_SWSCALE REQUIRED libswscale>=6)
# pkg_check_modules(ffmpeg_LIBS REQUIRED IMPORTED_TARGET libavcodec libavformat libavutil)


add_library(
  video_encoder SHARED
  src/video_encoder.cpp
)
# target_link_libraries(video_encoder avcodec avformat avutil)
# target_link_libraries(video_encoder ${ffmpeg_LIBRARIES})
link_directories(${ffmpeg_libs_DIR})
target_link_libraries(
  video_encoder
  ${PC_AVFORMAT_LIBRARIES}
  ${PC_AVCODEC_LIBRARIES}
  ${PC_AVUTIL_LIBRARIES}
  ${PC_SWSCALE_LIBRARIES}
)

message(WARNING "zczjx--> ")
message(WARNING "ffmpeg_libs_DIR: ${ffmpeg_libs_DIR} ")
message(WARNING "PC_AVFORMAT_LIBRARIES: ${PC_AVFORMAT_LIBRARIES} ")
message(WARNING "PC_AVCODEC_LIBRARIES: ${PC_AVCODEC_LIBRARIES} ")

target_include_directories(video_encoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ffmpeg_headers_DIR})
ament_target_dependencies(
  video_encoder
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)



add_executable(
  video_enc_node
  src/video_enc_node.cpp
)
# target_link_libraries(video_enc_node avcodec avfilter avformat avutil)
link_directories(${ffmpeg_libs_DIR})
target_link_libraries(
  video_enc_node
  video_encoder
  ${PC_AVFORMAT_LIBRARIES}
  ${PC_AVCODEC_LIBRARIES}
  ${PC_AVUTIL_LIBRARIES}
  ${PC_SWSCALE_LIBRARIES}
)

target_include_directories(video_enc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ffmpeg_headers_DIR})
ament_target_dependencies(
  video_enc_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

install(TARGETS video_enc_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()