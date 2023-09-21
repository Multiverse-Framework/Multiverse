add_executable(multiverse_server ${CMAKE_CURRENT_SOURCE_DIR}/src/multiverse_server.cpp)

if(UNIX)
    target_link_libraries(multiverse_server zmq jsoncpp pthread)
    install(TARGETS multiverse_server DESTINATION ${BIN_DIR})
elseif(WIN32)
    target_link_libraries(multiverse_server PRIVATE cppzmq cppzmq-static JsonCpp::JsonCpp)
endif()