aux_source_directory(. DIR_SRCS)
aux_source_directory(raw DIR_RAW_SRCS)
aux_source_directory(proto DIR_PROTO_SRCS)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

add_library(message SHARED ${DIR_SRCS} ${DIR_RAW_SRCS} ${DIR_PROTO_SRCS})
set_target_properties(message PROPERTIES OUTPUT_NAME "admessage")
set_target_properties(message PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(message PROPERTIES VERSION
    ${SONAME_MAJOR}.${SONAME_MINOR}.${SONAME_PATCH} SOVERSION ${SONAME_MAJOR})

if(SHARED_STATIC)
  add_library(message_static STATIC ${DIR_SRCS} ${DIR_RAW_SRCS} ${DIR_PROTO_SRCS})
  set_target_properties(message_static PROPERTIES OUTPUT_NAME "admessage")
  set_target_properties(message_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
endif()
