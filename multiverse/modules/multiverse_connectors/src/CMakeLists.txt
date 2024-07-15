if (UNIX)
    if(MULTIVERSE_CLIENT_LIBRARY_TYPE STREQUAL "STATIC" AND STDLIB STREQUAL "libstdc++")
        add_subdirectory(mujoco)
    endif()
endif()

add_subdirectory(multiverse_client_py)

add_subdirectory(ros)