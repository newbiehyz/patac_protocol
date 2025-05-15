ADD_EXECUTABLE(test_cross_correlation_key ${CMAKE_CURRENT_SOURCE_DIR}/test_cross_correlation_key.cc)

ADD_EXECUTABLE(test_traverse ${CMAKE_CURRENT_SOURCE_DIR}/test_traverse.cc)

ADD_EXECUTABLE(test_heatmap ${CMAKE_CURRENT_SOURCE_DIR}/test_heatmap.cc)
TARGET_LINK_LIBRARIES(test_heatmap ${PROJECT_NAME})


ADD_EXECUTABLE(test_noise ${CMAKE_CURRENT_SOURCE_DIR}/test_noise.cc)
