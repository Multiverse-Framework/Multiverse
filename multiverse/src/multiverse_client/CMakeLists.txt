if(WIN32)
    find_package(cppzmq REQUIRED)
    find_package(jsoncpp REQUIRED)
endif()

set(PYBIND11_FINDPYTHON ON)
if (EXISTS ${BUILD_DIR}/pybind11/share/cmake/pybind11)
    set(pybind11_DIR ${BUILD_DIR}/pybind11/share/cmake/pybind11)
endif()

find_program(Python_EXECUTABLE ${PYTHON_EXECUTABLE})
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)

set(MULTIVERSE_CLIENT multiverse_client)
set(MULTIVERSE_CLIENT_JSON multiverse_client_json)
set(MULTIVERSE_CLIENT_PYBIND multiverse_client_pybind)

if(UNIX)
    if(NOT MULTIVERSE_CLIENT_LIBRARY_TYPE STREQUAL "SHARED" AND NOT MULTIVERSE_CLIENT_LIBRARY_TYPE STREQUAL "STATIC")
        message(FATAL_ERROR "MULTIVERSE_CLIENT_LIBRARY_TYPE must be either SHARED or STATIC")
    endif()

    if(NOT STDLIB STREQUAL "libc++" AND NOT STDLIB STREQUAL "libstdc++")
        message(FATAL_ERROR "STDLIB must be either libc++ or libstdc++")
    endif()

    execute_process(
        COMMAND lsb_release -rs
        OUTPUT_VARIABLE UBUNTU_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if (UBUNTU_VERSION STREQUAL "20.04")
        set(CMAKE_CXX_COMPILER "/usr/bin/clang++-11")
        set(CMAKE_CXX_STANDARD 11)
    elseif (UBUNTU_VERSION STREQUAL "22.04")
        set(CMAKE_CXX_COMPILER "/usr/bin/clang++-14")
        set(CMAKE_CXX_STANDARD 14)
    elseif (UBUNTU_VERSION STREQUAL "24.04")
        set(CMAKE_CXX_COMPILER "/usr/bin/clang++-17")
        set(CMAKE_CXX_STANDARD 14)
    endif()
endif()

function(build_multiverse_client)
    if (UNIX)
        add_library(${MULTIVERSE_CLIENT} ${MULTIVERSE_CLIENT_LIBRARY_TYPE} ${CMAKE_CURRENT_SOURCE_DIR}/src/${MULTIVERSE_CLIENT}.cpp)
    elseif (WIN32)
        add_library(${MULTIVERSE_CLIENT} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/${MULTIVERSE_CLIENT}.cpp)
    endif()
    
    target_include_directories(${MULTIVERSE_CLIENT} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_directories(${MULTIVERSE_CLIENT} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

    if (UNIX)
        target_link_libraries(${MULTIVERSE_CLIENT} PRIVATE c++abi zmq atomic)
        target_compile_options(${MULTIVERSE_CLIENT} PRIVATE ${BUILD_FLAGS})
        install(TARGETS ${MULTIVERSE_CLIENT} DESTINATION ${LIB_DIR}/${STDLIB})
    elseif (WIN32)
        target_link_libraries(${MULTIVERSE_CLIENT} PRIVATE cppzmq)
        install(TARGETS ${MULTIVERSE_CLIENT} DESTINATION ${LIB_DIR})
    endif()
endfunction()

function(build_multiverse_client_json)
    if (UNIX)
        add_library(${MULTIVERSE_CLIENT_JSON} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/${MULTIVERSE_CLIENT_JSON}.cpp)
    elseif (WIN32)
        add_library(${MULTIVERSE_CLIENT_JSON} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/${MULTIVERSE_CLIENT_JSON}.cpp)
    endif()

    target_include_directories(${MULTIVERSE_CLIENT_JSON} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_directories(${MULTIVERSE_CLIENT_JSON} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

    if (UNIX)
        target_link_libraries(${MULTIVERSE_CLIENT_JSON} PRIVATE ${MULTIVERSE_CLIENT} jsoncpp)
        target_compile_options(${MULTIVERSE_CLIENT_JSON} PRIVATE ${BUILD_FLAGS})
        install(TARGETS ${MULTIVERSE_CLIENT_JSON} DESTINATION ${LIB_DIR}/${STDLIB})
    elseif (WIN32)
        target_link_libraries(${MULTIVERSE_CLIENT_JSON} ${MULTIVERSE_CLIENT} JsonCpp::JsonCpp)
        install(TARGETS ${MULTIVERSE_CLIENT_JSON} DESTINATION ${LIB_DIR})
    endif()
endfunction()

function(buid_multiverse_client_pybind)
    pybind11_add_module(${MULTIVERSE_CLIENT_PYBIND} MODULE ${CMAKE_CURRENT_SOURCE_DIR}/src/${MULTIVERSE_CLIENT_PYBIND}.cpp)
    target_include_directories(${MULTIVERSE_CLIENT_PYBIND} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_directories(${MULTIVERSE_CLIENT_PYBIND} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${MULTIVERSE_CLIENT_PYBIND} PRIVATE ${MULTIVERSE_CLIENT})
    install(TARGETS ${MULTIVERSE_CLIENT_PYBIND} DESTINATION ${LIB_DIR}/dist-packages)
endfunction()

if (UNIX)
    if(STDLIB STREQUAL "libstdc++")
        set(BUILD_FLAGS "-fPIC")
    else()
        set(BUILD_FLAGS "-stdlib=${STDLIB}")
    endif()    

    include_directories(
        "/usr/include/c++/${CMAKE_CXX_STANDARD}"
        "/usr/include/x86_64-linux-gnu/c++/${CMAKE_CXX_STANDARD}"
    )

    build_multiverse_client()

    if(MULTIVERSE_CLIENT_LIBRARY_TYPE STREQUAL "STATIC" AND STDLIB STREQUAL "libstdc++")
        build_multiverse_client_json()
    endif()

    if(MULTIVERSE_CLIENT_LIBRARY_TYPE STREQUAL "STATIC" AND STDLIB STREQUAL "libstdc++")
        buid_multiverse_client_pybind()
    endif()
elseif(WIN32)
    build_multiverse_client()

    build_multiverse_client_json()

    buid_multiverse_client_pybind()
endif()
