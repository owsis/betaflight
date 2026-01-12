cmake_minimum_required(VERSION 3.5)
project(betaflight_gazebo_plugin)

# Find required packages
find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)

# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link directories
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Common libraries for all plugins
set(COMMON_LIBS
  ${GAZEBO_LIBRARIES}
  ignition-math6
)

# ============================================================
# Dynamically find and build all plugin source files
# Pattern: *Plugin*.cc
# ============================================================

file(GLOB PLUGIN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*Plugin*.cc")

message(STATUS "===========================================")
message(STATUS "Betaflight Gazebo Plugin Build System")
message(STATUS "===========================================")
message(STATUS "Gazebo version: ${GAZEBO_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "")
message(STATUS "Found plugin sources:")

foreach(PLUGIN_SOURCE ${PLUGIN_SOURCES})
  # Get filename without extension
  get_filename_component(PLUGIN_NAME ${PLUGIN_SOURCE} NAME_WE)

  message(STATUS "  - ${PLUGIN_NAME}")

  # Create shared library for each plugin
  add_library(${PLUGIN_NAME} SHARED ${PLUGIN_SOURCE})
  target_link_libraries(${PLUGIN_NAME} ${COMMON_LIBS})

  # Set output name (lib prefix is added automatically)
  set_target_properties(${PLUGIN_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    OUTPUT_NAME ${PLUGIN_NAME}
  )

  # Add to install targets
  install(TARGETS ${PLUGIN_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endforeach()

message(STATUS "")
message(STATUS "===========================================")

# ============================================================
# Copy plugins to source directory for easy access
# ============================================================

foreach(PLUGIN_SOURCE ${PLUGIN_SOURCES})
  get_filename_component(PLUGIN_NAME ${PLUGIN_SOURCE} NAME_WE)

  add_custom_command(TARGET ${PLUGIN_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
      $<TARGET_FILE:${PLUGIN_NAME}>
      ${CMAKE_CURRENT_SOURCE_DIR}/lib${PLUGIN_NAME}.so
    COMMENT "Copying ${PLUGIN_NAME} to source directory"
  )
endforeach()

# ============================================================
# Print usage information
# ============================================================

message(STATUS "")
message(STATUS "Build commands:")
message(STATUS "  mkdir -p build && cd build")
message(STATUS "  cmake ..")
message(STATUS "  make -j\$(nproc)")
message(STATUS "")
message(STATUS "After build, plugins will be copied to:")
message(STATUS "  ${CMAKE_CURRENT_SOURCE_DIR}/")
message(STATUS "")
message(STATUS "To use with Gazebo, set:")
message(STATUS "  export GAZEBO_PLUGIN_PATH=${CMAKE_CURRENT_SOURCE_DIR}:\$GAZEBO_PLUGIN_PATH")
message(STATUS "")
