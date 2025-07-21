cmake_minimum_required(VERSION 3.10)
project(patac_hpp_protocol)

set(CMAKE_CXX_STANDARD 17)

find_package(SQLite3 REQUIRED)
include_directories(${SQLite3_INCLUDE_DIRS})
# target_link_libraries(${SQLite3_LIBRARIES})


find_package(Protobuf REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")
include_directories(${Protobuf_INCLUDE_DIRS})
# target_link_libraries(${Protobuf_LIBRARIES})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# target_link_libraries(${OpenCV_LIBS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


file(GLOB PROTOCOL_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/cpp_gen/*.cc")
message(PROTOCOL_SOURCE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cpp_gen)

add_library(patac_protocol SHARED ${PROTOCOL_SOURCE})
target_link_libraries(patac_protocol ${Protobuf_LIBRARIES})

add_library(patac_data_writer SHARED ${CMAKE_CURRENT_SOURCE_DIR}/data_writer.cc)
target_link_libraries(patac_data_writer patac_protocol ${Protobuf_LIBRARIES} ${SQLite3_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_writer test_writer.cc)
target_link_libraries(test_writer patac_data_writer ${Protobuf_LIBRARIES} ${SQLite3_LIBRARIES} ${OpenCV_LIBS})

