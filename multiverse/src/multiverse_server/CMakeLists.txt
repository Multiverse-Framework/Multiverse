add_executable(multiverse_server ${CMAKE_CURRENT_SOURCE_DIR}/src/multiverse_server.cpp)

target_link_libraries(multiverse_server zmq jsoncpp pthread)

install(TARGETS multiverse_server DESTINATION ${BIN_DIR})