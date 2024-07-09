project(multiverse_source)

if(NOT WIN32) # Skip this because cppzmq in vcpgk is not working
    add_subdirectory(multiverse_server)
endif()

add_subdirectory(multiverse_client)