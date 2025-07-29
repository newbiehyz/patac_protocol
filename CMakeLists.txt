cmake_minimum_required(VERSION 3.10)
project(patac_hpp_protocol)

set(CMAKE_CXX_STANDARD 17)

# find_package(SQLite3 REQUIRED)
# include_directories(${SQLite3_INCLUDE_DIRS})
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

add_library(sqlite3 STATIC sqlite3.c)
target_link_libraries(sqlite3 Threads::Threads)
target_include_directories(sqlite3 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(sqlite3 PROPERTIES POSITION_INDEPENDENT_CODE ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cpp_gen)

add_library(patac_protocol SHARED ${PROTOCOL_SOURCE})
target_link_libraries(patac_protocol ${Protobuf_LIBRARIES})

# add_library(patac_data_writer SHARED ${CMAKE_CURRENT_SOURCE_DIR}/data_writer.cc)
# target_link_libraries(patac_data_writer patac_protocol ${Protobuf_LIBRARIES} ${OpenCV_LIBS} sqlite3)

add_executable(test_writer test_writer.cc data_writer.cc)
target_link_libraries(test_writer ${Protobuf_LIBRARIES} ${OpenCV_LIBS} sqlite3 Threads::Threads dl patac_protocol)

add_executable(test_reader test_reader.cc data_reader.cc)
target_link_libraries(test_reader ${Protobuf_LIBRARIES} ${OpenCV_LIBS} sqlite3 Threads::Threads dl patac_protocol)

