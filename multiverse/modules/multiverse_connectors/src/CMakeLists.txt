if(MULTIVERSE_CLIENT_LIBRARY_TYPE STREQUAL "STATIC" AND STDLIB STREQUAL "libstdc++")
    add_subdirectory(mujoco)
endif()

# if (UNIX)
#     add_subdirectory(ros)
# endif()

add_subdirectory(multiverse_client_py)