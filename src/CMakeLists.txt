aux_source_directory(. DIRSRCS)
aux_source_directory(utils UTILS)
aux_source_directory(infra/framework INFRA_FRAMEWORK_DIRSRCS)
aux_source_directory(type VISION_DEBUG_INFO)
aux_source_directory(type/serializer PROTOCOL_TYPE_SER_DIRSRCS)
aux_source_directory(foundation/message IFRA_PROTOCOL_MESSAGE_DIRSRCS)
aux_source_directory(interface INTERFACE_DIRSRCS)
aux_source_directory(interface/parking INTERFACE_PARKING_DIRSRCS)
aux_source_directory(interface/parking/serializer PROTOCOL_INTERFACE_PARKING_SERIALIZER_DIRSRCS)
aux_source_directory(interface/serializer PROTOCOL_INTERFACE_SER_DIRSRCS)
aux_source_directory(model MODEL_DIRSRCS)
aux_source_directory(model/serializer PROTOCOL_MODEL_SER_DIRSRCS)
aux_source_directory(foundation/type MSG_DIRSRCS)
aux_source_directory(protocol PROTOCOL_PROTOCOL_DIRSRCS)
aux_source_directory(serializer PROTOCOL_SERIALIZER_DIRSRCS)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
#add_library(template SHARED ${DIRSRCS} ${VERSION_DIRSRCS})
#set_target_properties(template PROPERTIES OUTPUT_NAME "adtemplate")
#set_target_properties(template PROPERTIES CLEAN_DIRECT_OUTPUT 1)
#set_target_properties(template PROPERTIES VERSION
   # ${SONAME_MAJOR}.${SONAME_MINOR}.${SONAME_PATCH} SOVERSION ${SONAME_MAJOR})

if(SHARED_STATIC)
  add_library(DATAFLOW_static STATIC ${DIRSRCS} ${INFRA_FRAMEWORK_DIRSRCS} ${UTILS}
  ${VISION_DEBUG_INFO} ${PROTOCOL_TYPE_SER_DIRSRCS}
  ${IFRA_PROTOCOL_MESSAGE_DIRSRCS} ${INTERFACE_DIRSRCS} ${INTERFACE_PARKING_DIRSRCS}
  ${PROTOCOL_INTERFACE_PARKING_SERIALIZER_DIRSRCS}
  ${PROTOCOL_INTERFACE_SER_DIRSRCS} ${MODEL_DIRSRCS} ${PROTOCOL_MODEL_SER_DIRSRCS}
  ${MSG_DIRSRCS} ${PROTOCOL_PROTOCOL_DIRSRCS} ${PROTOCOL_SERIALIZER_DIRSRCS})
  set_target_properties(DATAFLOW_static PROPERTIES OUTPUT_NAME "hyper-protocol")
  set_target_properties(DATAFLOW_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
endif()

add_library(com SHARED ${DIRSRCS} ${INFRA_FRAMEWORK_DIRSRCS} ${UTILS} ${IFRA_PROTOCOL_MESSAGE_DIRSRCS}
  ${VISION_DEBUG_INFO} ${PROTOCOL_TYPE_SER_DIRSRCS}
  ${INTERFACE_DIRSRCS} ${INTERFACE_PARKING_DIRSRCS} ${PROTOCOL_INTERFACE_PARKING_SERIALIZER_DIRSRCS}
  ${PROTOCOL_INTERFACE_SER_DIRSRCS} ${MODEL_DIRSRCS} ${PROTOCOL_MODEL_SER_DIRSRCS}
  ${MSG_DIRSRCS} ${PROTOCOL_PROTOCOL_DIRSRCS} ${PROTOCOL_SERIALIZER_DIRSRCS})
#add_library(com_static STATIC ${DIR_SRCS} ${INTERFACE_PARKING_DIRSRCS} ${MSG_DIRSRCS} ${MODEL_DIRSRCS})
set_target_properties(com PROPERTIES OUTPUT_NAME "hyper-protocol")
#set_target_properties(com_static PROPERTIES OUTPUT_NAME "hyper-protocol")
#set_target_properties(com_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(com PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(com PROPERTIES VERSION ${SONAME_MAJOR}.${SONAME_MINOR}.${SONAME_PATCH} SOVERSION ${SONAME_MAJOR})
add_executable(	demo ${DIRSRCS})
add_dependencies(demo com)
target_link_libraries(
  demo hyper-protocol protobuf)
