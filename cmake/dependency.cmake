
message("compile dependency, path: ${CMAKE_DEPENDENCY_DIRECTORY}")

IF(DEFINED TEST_TOOLCHAIN_ROOT_10_3_0)
  include_directories(
    ${CMAKE_DEPENDENCY_DIRECTORY}/thirdparty/protobuf/3.5.0_linux_aarch64_gcc10.3.0/include
  )
  link_directories(
    ${CMAKE_DEPENDENCY_DIRECTORY}/thirdparty/protobuf/3.5.0_linux_aarch64_gcc10.3.0/lib
  )
ELSEIF(DEFINED TEST_TOOLCHAIN_ROOT)
  include_directories(
    ${CMAKE_DEPENDENCY_DIRECTORY}/thirdparty/protobuf/3.5.0_linux_aarch64_gcc6.5.0/include
  )
  link_directories(
    ${CMAKE_DEPENDENCY_DIRECTORY}/thirdparty/protobuf/3.5.0_linux_aarch64_gcc6.5.0/lib
  )
ELSE()
  include_directories(
    ${CMAKE_DEPENDENCY_DIRECTORY}/thirdparty/protobuf/3.5.0_linux_ubuntu_gcc9.4.0/include
  )
  link_directories(
    ${CMAKE_DEPENDENCY_DIRECTORY}/thirdparty/protobuf/3.5.0_linux_ubuntu_gcc9.4.0/lib
  )
ENDIF()
