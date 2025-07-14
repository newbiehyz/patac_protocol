cmake_minimum_required(VERSION 3.5.1)
project("autodrive" VERSION 2022 LANGUAGES CXX C)

# ==== 工具链配置（保留 ARM 支持）====
IF(DEFINED TEST_TOOLCHAIN_ROOT_10_3_0)
  message("compile ARM, toolchain 10.3.0")
  set(CMAKE_SYSTEM_NAME Linux)
  set(CMAKE_SYSTEM_PROCESSOR arm)
  set(TEST_COMPILER_ROOT "${TEST_TOOLCHAIN_ROOT_10_3_0}/gcc-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin")
  set(CMAKE_C_COMPILER ${TEST_COMPILER_ROOT}/aarch64-none-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER ${TEST_COMPILER_ROOT}/aarch64-none-linux-gnu-g++)
  set(CMAKE_SYSROOT ${TEST_TOOLCHAIN_ROOT_10_3_0}/targetfs)
ELSEIF(DEFINED TEST_TOOLCHAIN_ROOT)
  message("compile ARM, toolchain 6.5.0")
  set(CMAKE_SYSTEM_NAME Linux)
  set(CMAKE_SYSTEM_PROCESSOR arm)
  set(TEST_COMPILER_ROOT "${TEST_TOOLCHAIN_ROOT}/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin")
  set(CMAKE_C_COMPILER ${TEST_COMPILER_ROOT}/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER ${TEST_COMPILER_ROOT}/aarch64-linux-gnu-g++)
  set(CMAKE_SYSROOT ${TEST_TOOLCHAIN_ROOT}/targetfs)
ENDIF()

set(CMAKE_CXX_STANDARD 14)
add_definitions("-Wall -O2")

# ==== Protobuf 配置 ====
set(PROTOBUF_ROOT /usr/local/bin/)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/protocol
  ${PROTOBUF_ROOT}/include
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${PROTOBUF_ROOT}/lib
)

# ==== 编译主程序 ====
file(GLOB PATAC_PROTO_SRCS
  ${PROJECT_SOURCE_DIR}/src/protocol/*.pb.cc
)

add_executable(demo
  ${PROJECT_SOURCE_DIR}/src/main.cpp
  ${PATAC_PROTO_SRCS}
)

target_link_libraries(demo
  protobuf
)

# ==== 安装（可选）====
option(INSTALL "install" ON)
if(INSTALL)
  set(OUTPUT "${PROJECT_SOURCE_DIR}/build/output/dataflow")
  install(TARGETS demo DESTINATION ${OUTPUT}/bin)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION ${OUTPUT}/include)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/lib/ DESTINATION ${OUTPUT}/lib)
endif()
