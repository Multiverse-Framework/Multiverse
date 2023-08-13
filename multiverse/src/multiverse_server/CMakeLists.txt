add_executable(multiverse_server src/multiverse_server.cpp)

target_link_libraries(multiverse_server zmq jsoncpp pthread)

install(TARGETS multiverse_server DESTINATION bin)