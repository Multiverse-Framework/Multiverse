project(multiverse_modules)

add_subdirectory(multiverse_connectors)

if (UNIX)
    add_subdirectory(multiverse_knowledge)

    add_subdirectory(multiverse_parser)
endif()