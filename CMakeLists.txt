cmake_minimum_required(VERSION 3.16)
project(mujoco_interface VERSION 0.2.0 LANGUAGES CXX)

include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT MUJOCO_DIR)
  set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mujoco" CACHE PATH "MuJoCo root")
endif()

if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
  message(FATAL_ERROR "MuJoCo not found at ${MUJOCO_DIR}. ln -s /opt/mujoco-3.3.6 mujoco")
endif()

set(ECAL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ecal/usr" CACHE PATH "Local eCAL prefix")
set(ECAL_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ecal/etc/ecal" CACHE PATH "Bundled eCAL config")
set(ECAL_LIB_DIR "${ECAL_ROOT}/lib/x86_64-linux-gnu")
if(NOT EXISTS "${ECAL_LIB_DIR}/libecal_core.so")
  message(FATAL_ERROR "eCAL not found. Run scripts/fetch_ecal.sh")
endif()

find_package(Threads REQUIRED)
find_package(glfw3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(ecal_core SHARED IMPORTED GLOBAL)
set_target_properties(ecal_core PROPERTIES
  IMPORTED_LOCATION "${ECAL_LIB_DIR}/libecal_core.so"
  INTERFACE_INCLUDE_DIRECTORIES "${ECAL_ROOT}/include"
)

file(GLOB SIMULATE_SRC
  ${MUJOCO_DIR}/simulate/glfw_*.cc
  ${MUJOCO_DIR}/simulate/platform_*.cc
  ${MUJOCO_DIR}/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

add_library(mujoco_interface_core
  src/config_loader.cpp
  src/robot_interface.cpp
  src/input.cpp
  src/core/clock.cpp
  src/core/client_registry.cpp
  src/core/command_arbiter.cpp
  src/core/barrier_manager.cpp
  src/core/model_manager.cpp
  src/core/simulation.cpp
  src/transport/ecal.cpp
  src/viewer/sim_runner.cpp
  ${SIMULATE_SRC}
)

target_include_directories(mujoco_interface_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${MUJOCO_DIR}/include>
  $<BUILD_INTERFACE:${MUJOCO_DIR}/simulate>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/lodepng>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mujoco_interface_core PUBLIC
  yaml-cpp
  Threads::Threads
  glfw
  ecal_core
  $<BUILD_INTERFACE:${MUJOCO_DIR}/lib/libmujoco.so>
)
target_compile_definitions(mujoco_interface_core PRIVATE
  MUJOCO_INTERFACE_ECAL_DATA="${ECAL_DATA_DIR}"
)

add_executable(mujoco_interface src/main.cpp)
target_link_libraries(mujoco_interface PRIVATE mujoco_interface_core)

add_executable(test_core tests/test_core.cpp)
target_link_libraries(test_core PRIVATE mujoco_interface_core)

set(ECAL_LIB_PATH "${ECAL_LIB_DIR}")
set_target_properties(mujoco_interface_core mujoco_interface PROPERTIES
  BUILD_RPATH "${ECAL_LIB_PATH};${MUJOCO_DIR}/lib"
  INSTALL_RPATH "$ORIGIN/../lib"
)
enable_testing()
add_test(NAME core COMMAND test_core)

install(TARGETS mujoco_interface_core mujoco_interface
  EXPORT mujoco_interfaceTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/mujoco_interface DESTINATION include)
install(DIRECTORY "${MUJOCO_DIR}/include/" DESTINATION include)
install(DIRECTORY "${ECAL_ROOT}/include/" DESTINATION include)
install(FILES
  "${MUJOCO_DIR}/lib/libmujoco.so"
  "${MUJOCO_DIR}/lib/libmujoco.so.3.3.6"
  "${ECAL_LIB_DIR}/libecal_core.so"
  "${ECAL_LIB_DIR}/libecal_core.so.6"
  "${ECAL_LIB_DIR}/libecal_core.so.6.1.1"
  DESTINATION lib
)
install(DIRECTORY "${ECAL_DATA_DIR}/" DESTINATION share/mujoco_interface/ecal)

install(EXPORT mujoco_interfaceTargets
  FILE mujoco_interfaceTargets.cmake
  NAMESPACE mujoco_interface::
  DESTINATION lib/cmake/mujoco_interface
)

configure_package_config_file(
  cmake/mujoco_interfaceConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_interfaceConfig.cmake"
  INSTALL_DESTINATION lib/cmake/mujoco_interface
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_interfaceConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_interfaceConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/mujoco_interfaceConfigVersion.cmake"
  DESTINATION lib/cmake/mujoco_interface
)
