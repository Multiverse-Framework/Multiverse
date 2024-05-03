project(multiverse_modules)

option(BUILD_CONNECTORS "Build the multiverse connectors" ON)
option(BUILD_KNOWLEDGE "Build the multiverse knowledge" ON)
option(BUILD_PARSER "Build the multiverse parser" ON)

if(BUILD_CONNECTORS)
    add_subdirectory(multiverse_connectors)
endif()

if(UNIX)
    if(BUILD_KNOWLEDGE)
        add_subdirectory(multiverse_knowledge)
    endif()

    if(BUILD_PARSER)
        add_subdirectory(multiverse_parser)
    endif()
endif()