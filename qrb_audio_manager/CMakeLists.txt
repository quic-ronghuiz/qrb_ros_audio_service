cmake_minimum_required(VERSION 3.8)
project(qrb_audio_manager VERSION 1.0.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(SOURCES
  src/audio_manager.cpp
  src/message_queue.cpp
  src/stream.cpp
  src/playback_stream.cpp
  src/record_stream.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Config
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(EXPORT ${PROJECT_NAME}Config
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_auto_package()