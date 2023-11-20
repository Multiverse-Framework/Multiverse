project(multiverse_source)

if(WIN32)
    find_package(jsoncpp REQUIRED)
    find_package(Zeromq REQUIRED)
    find_package(cppzmq REQUIRED)
    include_directories(${BUILD_DIR}/vcpkg_installed/x64-windows/include)
endif()

add_subdirectory(multiverse_server)

add_subdirectory(multiverse_client)