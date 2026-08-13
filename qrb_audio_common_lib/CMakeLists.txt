cmake_minimum_required(VERSION 3.8)
project(qrb_audio_common_lib VERSION 2.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(PkgConfig REQUIRED)

pkg_check_modules(ALSA REQUIRED alsa)
pkg_check_modules(SNDFILE REQUIRED sndfile)

add_library(${PROJECT_NAME} SHARED
  src/audio_stream.cpp
  src/alsa_stream.cpp
  src/pulse_stream.cpp
  src/audio_common_wrapper.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${ALSA_LIBRARIES}
    ${SNDFILE_LIBRARIES}
    pulse
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

install(FILES
  include/${PROJECT_NAME}/audio_common_wrapper.hpp
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Config
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(EXPORT ${PROJECT_NAME}Config
  DESTINATION share/${PROJECT_NAME}/cmake
)
