# References:
# https://stackoverflow.com/questions/56135785/correctly-set-the-location-of-imported-cmake-targets-for-an-installed-package

cmake_minimum_required(VERSION 3.5)
project(go2_hal)

# Ensures CMake creates the file compile_commands.json (Required for Vim-YCM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_FIND_DEBUG_MODE FALSE)

set(VERSION 0.2.0)

option(BUILD_GO2HAL_TEST "Build the tests" ON)
option(BUILD_GO2HAL_PYTHON "Build the python bindings (pygo2_hal)" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(INCLUDE_INSTALL_DIR include)  # include
set(LIBRARY_INSTALL_DIR lib)
set(ConfigFileInstallDir lib/cmake/go2hal)


# ###################
# GO2 HAL LIBRARY
# ###################

# unitree_sdk2 dependency (DDS-based SDK for Go2/B2/H1/...).
# NOTE: this replaces the legacy unitree_legged_sdk (UDP-based, Go1-era) dependency.
find_package(unitree_sdk2 QUIET)
if(NOT unitree_sdk2_FOUND)
  # Compile and install unitree_sdk2 if it is not already installed
  add_subdirectory(third_party/unitree_sdk2)
endif()

add_library(go2hal SHARED
  src/go2_hal.cpp
)

target_link_libraries(go2hal
  PUBLIC
  unitree_sdk2
)

target_include_directories(go2hal
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>   # <prefix>/include
)

set_property(TARGET go2hal PROPERTY CXX_STANDARD 17)  # unitree_sdk2 / cyclonedds-cxx require >= C++17

# # TODO: this is not necessary when using a shared library. Uncomment it if we decide to make the library static
# set_property(TARGET go2hal PROPERTY POSITION_INDEPENDENT_CODE ON)


# ##############
# PY GO2 HAL
# ##############
# TODO: python_interface.cpp still binds the *old* unitree_legged_sdk-style
# structs (plain public members, e.g. MotorState::q). It needs to be rewritten
# for unitree_sdk2's IDL-generated accessor-method classes (e.g.
# unitree_go::msg::dds_::MotorState_::q()) before this can be re-enabled.
if(BUILD_GO2HAL_PYTHON)
  add_subdirectory(third_party/pybind11)

  pybind11_add_module(pygo2_hal
    src/python_interface.cpp
  )
  target_link_libraries(pygo2_hal
    PUBLIC
      go2hal
      pybind11::embed
  )
endif()


# #####
# TESTS
# #####
if(BUILD_GO2HAL_TEST)
  add_subdirectory(test)
endif()


# #######
# INSTALL
# #######
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/include/
  DESTINATION include
)
install(TARGETS
  go2hal
  EXPORT go2halTargets
  RUNTIME DESTINATION bin COMPONENT Runtime
  LIBRARY DESTINATION lib COMPONENT Runtime
  ARCHIVE DESTINATION lib COMPONENT Development
  PUBLIC_HEADER DESTINATION include COMPONENT Development
  BUNDLE DESTINATION bin COMPONENT Runtime
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/go2halConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/go2halConfig.cmake"
  INSTALL_DESTINATION "${ConfigFileInstallDir}"
  PATH_VARS INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR
)
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/go2halConfigVersion.cmake"
  VERSION ${VERSION}
  COMPATIBILITY AnyNewerVersion
)
