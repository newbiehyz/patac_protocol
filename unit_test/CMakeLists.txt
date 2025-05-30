ADD_EXECUTABLE(test_cross_correlation_key ${CMAKE_CURRENT_SOURCE_DIR}/test_cross_correlation_key.cc)

ADD_EXECUTABLE(test_traverse ${CMAKE_CURRENT_SOURCE_DIR}/test_traverse.cc)

ADD_EXECUTABLE(test_heatmap ${CMAKE_CURRENT_SOURCE_DIR}/test_heatmap.cc)
TARGET_LINK_LIBRARIES(test_heatmap ${PROJECT_NAME})


ADD_EXECUTABLE(test_noise ${CMAKE_CURRENT_SOURCE_DIR}/test_noise.cc)

ADD_EXECUTABLE(test_steering_angle ${CMAKE_CURRENT_SOURCE_DIR}/test_steering_angle.cc)

ADD_EXECUTABLE(test_udp_send ${CMAKE_CURRENT_SOURCE_DIR}/test_udp_send.cc)




IF(ENABLE_OPENGL) 

  ADD_EXECUTABLE(test_udp_recv ${CMAKE_CURRENT_SOURCE_DIR}/test_udp_recv.cc)
  TARGET_LINK_LIBRARIES(test_udp_recv  ${PROJECT_NAME} ${Pangolin_LIBRARIES})

  ADD_EXECUTABLE(debug_main ${CMAKE_CURRENT_SOURCE_DIR}/debug_main.cc)
  TARGET_LINK_LIBRARIES(debug_main ${PROJECT_NAME} ${Pangolin_LIBRARIES})

ENDIF()


