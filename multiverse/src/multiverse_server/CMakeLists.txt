add_executable(multiverse_server ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_include_directories(multiverse_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(multiverse_server_lib ${CMAKE_CURRENT_SOURCE_DIR}/src/multiverse_server.cpp)
target_include_directories(multiverse_server_lib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_directories(multiverse_server_lib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(UNIX)
    target_link_libraries(multiverse_server multiverse_server_lib zmq jsoncpp pthread)
elseif(WIN32)
    target_link_libraries(multiverse_server multiverse_server_lib cppzmq cppzmq-static JsonCpp::JsonCpp)
endif()

install(TARGETS multiverse_server DESTINATION ${BIN_DIR})
