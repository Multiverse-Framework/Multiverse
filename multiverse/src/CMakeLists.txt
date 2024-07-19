project(multiverse_source)

if (NOT CMAKE_TOOLCHAIN_FILE MATCHES "vcpkg.cmake")
    add_subdirectory(multiverse_server)
endif()

add_subdirectory(multiverse_client)