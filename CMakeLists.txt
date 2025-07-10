cmake_minimum_required(VERSION 3.5.1)

project("autodrive" VERSION 2022 LANGUAGES CXX C)

set(CMAKE_DEPENDENCY_DIRECTORY ${PROJECT_SOURCE_DIR}/../dependency)
include(${PROJECT_SOURCE_DIR}/cmake/dependency.cmake)

# === ARM Toolchain Handling ===
IF(DEFINED TEST_TOOLCHAIN_ROOT_10_3_0)
  message("compile ARM, targetf ${TEST_TOOLCHAIN_ROOT_10_3_0}")
  add_definitions("-O2")
  set(TEST_COMPILER_ROOT "${TEST_TOOLCHAIN_ROOT_10_3_0}/gcc-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin")

  set(CMAKE_SYSTEM_NAME Linux)
  set(CMAKE_SYSTEM_PROCESSOR arm)
  set(CMAKE_CXX_COMPILER ${TEST_COMPILER_ROOT}/aarch64-none-linux-gnu-g++)
  set(CMAKE_C_COMPILER  ${TEST_COMPILER_ROOT}/aarch64-none-linux-gnu-gcc)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS} -O0 -Wall -g2 -ggdb")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -Wall")

ELSEIF(DEFINED TEST_TOOLCHAIN_ROOT)
  message("compile ARM, targetf ${TEST_TOOLCHAIN_ROOT}")
  add_definitions("-O2")
  set(TEST_COMPILER_ROOT "${TEST_TOOLCHAIN_ROOT}/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin")

  set(CMAKE_SYSTEM_NAME Linux)
  set(CMAKE_SYSTEM_PROCESSOR arm)
  set(CMAKE_CXX_COMPILER ${TEST_COMPILER_ROOT}/aarch64-linux-gnu-g++)
  set(CMAKE_C_COMPILER  ${TEST_COMPILER_ROOT}/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS} -O0 -Wall -g2 -ggdb")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -Wall")
  set(CMAKE_SYSROOT ${TEST_TOOLCHAIN_ROOT}/targetfs)

ELSE()
  add_definitions("-std=c++14 -Wall -O2")
ENDIF()

# === Option Flags ===
option(SHARED_STATIC "shared static" OFF)
option(INSTALL "install" ON)

# === CCache & Version ===
include(${PROJECT_SOURCE_DIR}/cmake/ccache.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/version.cmake)

configure_file(
  ${PROJECT_SOURCE_DIR}/src/version/version.h.in
  ${PROJECT_SOURCE_DIR}/include/version/version.h @ONLY
)

message("Project source is at ${PROJECT_SOURCE_DIR}")

# === Protobuf Include & Lib Paths ===
set(PROTOBUF_ROOT /home/hyz/05_LYK/thirdparty/protobuf-3.5.0)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/protocol
  ${PROTOBUF_ROOT}/include
)

link_directories(
  ${PROJECT_BINARY_DIR}/lib
  ${PROTOBUF_ROOT}/lib
)

# === Build Source ===
add_subdirectory(${PROJECT_SOURCE_DIR}/src)

# === Install Output ===
if(INSTALL)
  set(OUTPUT "${PROJECT_SOURCE_DIR}/build/output/dataflow")
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION ${OUTPUT}/include)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/lib/ DESTINATION ${OUTPUT}/lib/)
  install(FILES ${PROJECT_SOURCE_DIR}/build/version.json DESTINATION ${OUTPUT})
endif()
