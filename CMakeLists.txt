cmake_minimum_required(VERSION 3.8)
project(qrb_audio_common_lib VERSION 0.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# add targets
add_library(${PROJECT_NAME} SHARED
  src/audio_common.cpp
  src/playback_stream.cpp
  src/capture_stream.cpp
  src/audio_common_wrapper.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  pulse
  sndfile
)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)

# install headers
file(GLOB HEADER_FILES "include/${PROJECT_NAME}/*.hpp")
install(FILES ${HEADER_FILES}
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Config
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(EXPORT ${PROJECT_NAME}Config DESTINATION share/${PROJECT_NAME}/cmake)
