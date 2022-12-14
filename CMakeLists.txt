cmake_minimum_required(VERSION 3.17)

project(jetson_encoder_compressed_image_transport)

include(CheckLanguage)
check_language(CUDA)
if(NOT CMAKE_CUDA_COMPILER)
  message(WARNING "cuda not found")
  return()
endif()
	
enable_language(CUDA)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(CUDAToolkit)
# if(NOT TARGET CUDA::nvjpeg)
#   message(WARNING "nvjpeg not found")
#   return()
# endif()

include(FetchContent)
FetchContent_Declare(cuda-api-wrappers
    GIT_REPOSITORY https://github.com/eyalroz/cuda-api-wrappers.git
    GIT_TAG 831666a0bfd1af0f44f4fa234ee2d983d347fcaa # v0.6.1-rc1
)
FetchContent_MakeAvailable(cuda-api-wrappers)

add_library(${PROJECT_NAME} SHARED
  src/manifest.cpp
  src/jetson_encoder_compressed_publisher.cpp
  src/color_space.cu
  ${SYS_ROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvBuffer.cpp
  ${SYS_ROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvElement.cpp
  ${SYS_ROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvElementProfiler.cpp
  ${SYS_ROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvJpegEncoder.cpp
  ${SYS_ROOT}/usr/src/jetson_multimedia_api/samples/common/classes/NvLogging.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_include_directories(${PROJECT_NAME}
  PRIVATE SYSTEM
    "${SYS_ROOT}/usr/src/jetson_multimedia_api/include"
    "${SYS_ROOT}/usr/src/jetson_multimedia_api/include/libjpeg-8b"
)

target_link_directories(${PROJECT_NAME} PRIVATE ${SYSROOT}/usr/lib/aarch64-linux-gnu/tegra)
target_link_options(${PROJECT_NAME}
  PRIVATE
    "-Wl,-rpath-link=${SYSROOT}/lib/aarch64-linux-gnu"
    "-Wl,-rpath-link=${SYSROOT}/usr/lib/aarch64-linux-gnu"
    "-Wl,-rpath-link=${SYSROOT}/usr/lib/aarch64-linux-gnu/tegra"
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:-Wno-pedantic>
)

target_link_libraries(${PROJECT_NAME}
  CUDA::cudart
  CUDA::nppicc
  nvjpeg
  cuda-api-wrappers::runtime-and-driver
)

ament_target_dependencies(${PROJECT_NAME}
  "image_transport"
  "rclcpp"
  "pluginlib"
  "sensor_msgs"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "include/"
  DESTINATION include
)
pluginlib_export_plugin_description_file(image_transport jetson_encoder_compressed_plugins.xml)

ament_package()
