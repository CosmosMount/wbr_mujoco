cmake_minimum_required(VERSION 3.22)
project(wbr_mujoco)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
endif()

if(NOT MUJOCO_SIM_INSTALL_DIR)
  set(MUJOCO_SIM_INSTALL_DIR "${CMAKE_SOURCE_DIR}/../mujoco_sim_core/install" CACHE PATH
      "Path to colcon install prefix for mujoco_sim_core")
endif()
if(NOT ECAL_ROOT)
  set(ECAL_ROOT "${CMAKE_SOURCE_DIR}/../mujoco_interface/third_party/ecal/usr" CACHE PATH
      "Path to vendored eCAL install prefix (sibling mujoco_interface)")
endif()
list(APPEND CMAKE_PREFIX_PATH "${MUJOCO_SIM_INSTALL_DIR}" "${ECAL_ROOT}")
file(GLOB MUJOCO_SIM_PACKAGE_PREFIXES
  "${MUJOCO_SIM_INSTALL_DIR}/mujoco_sim_*"
)
list(APPEND CMAKE_PREFIX_PATH ${MUJOCO_SIM_PACKAGE_PREFIXES})
if(DEFINED ENV{CMAKE_PREFIX_PATH})
  list(APPEND CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
endif()

set(MUJOCO_SIM_CORE_SRC "${CMAKE_SOURCE_DIR}/../mujoco_sim_core/core/mujoco_sim_core")
set(MUJOCO_VENDOR_ROOT "${MUJOCO_SIM_CORE_SRC}/mujoco/linux/x86/mujoco-3.10.0")

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()
find_package(mujoco_sim_adapter_sdk CONFIG REQUIRED)
find_package(mujoco_sim_config CONFIG REQUIRED)
find_package(mujoco_sim_core CONFIG QUIET)

option(BUILD_CTRL "Build ctrl adapter executable" ON)
if(BUILD_CTRL AND NOT TARGET mujoco_sim::adapter_sdk)
  message(FATAL_ERROR
    "mujoco_sim::adapter_sdk target missing after finding mujoco_sim_adapter_sdk")
endif()

add_library(control_core INTERFACE)
target_include_directories(control_core INTERFACE controller/include)

if(BUILD_CTRL)
  add_executable(ctrl
    controller/src/main.cpp
    controller/src/app.cpp
    controller/src/config.cpp
    controller/src/sim/config.cpp
    controller/src/sim/interface_mapping.cpp
    controller/src/sim/input_script.cpp
    controller/src/sim/controller_pipeline.cpp
    controller/src/sim/tick_controller.cpp
    controller/src/sim/input_resolver.cpp
  )
  target_include_directories(ctrl PRIVATE controller/include)
  target_link_libraries(ctrl PRIVATE
    control_core
    mujoco_sim::adapter_sdk
    yaml-cpp
    Threads::Threads
  )
  set_target_properties(ctrl PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  )
  mujoco_sim_apply_runtime_rpath(ctrl)
endif()

add_executable(test_import tests/test_import.cpp controller/src/sim/config.cpp)
target_include_directories(test_import PRIVATE controller/include)
if(mujoco_sim_core_FOUND)
  target_link_libraries(test_import PRIVATE control_core yaml-cpp mujoco::mujoco)
else()
  target_include_directories(test_import PRIVATE "${MUJOCO_VENDOR_ROOT}/include")
  target_link_libraries(test_import PRIVATE control_core yaml-cpp
    "${MUJOCO_VENDOR_ROOT}/lib/libmujoco.so.3.10.0")
  set_target_properties(test_import PROPERTIES
    BUILD_RPATH "${MUJOCO_VENDOR_ROOT}/lib"
    INSTALL_RPATH "${MUJOCO_VENDOR_ROOT}/lib"
  )
endif()
set_target_properties(test_import PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

enable_testing()
add_executable(input_script_test tests/input_script_test.cpp controller/src/sim/input_script.cpp)
target_include_directories(input_script_test PRIVATE controller/include)
target_link_libraries(input_script_test PRIVATE control_core)
add_test(NAME input_script_test COMMAND input_script_test)
