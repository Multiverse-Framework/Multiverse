add_executable(multiverse_server ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

if(UNIX)
    add_library(multiverse_server_lib ${CMAKE_CURRENT_SOURCE_DIR}/src/multiverse_server.cpp)
    target_include_directories(multiverse_server_lib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_directories(multiverse_server_lib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)   

    target_include_directories(multiverse_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(multiverse_server multiverse_server_lib zmq jsoncpp pthread)
    install(TARGETS multiverse_server DESTINATION ${BIN_DIR})
elseif(WIN32)
    target_link_libraries(multiverse_server PRIVATE cppzmq cppzmq-static JsonCpp::JsonCpp)
endif()