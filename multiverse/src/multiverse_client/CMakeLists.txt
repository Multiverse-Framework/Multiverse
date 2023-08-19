set(MULTIVERSE_CLIENT multiverse_client)
set(MULTIVERSE_CLIENT_JSON multiverse_client_json)
set(MULTIVERSE_CLIENT_PYBIND multiverse_client_pybind)
set(STDLIBS "libc++" "libstdc++")

set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fPIC")

function(build_multiverse_client library_type)
    set(MULTIVERSE_CLIENT_LIB ${MULTIVERSE_CLIENT}_${STDLIB}_${library_type})
    add_library(${MULTIVERSE_CLIENT_LIB} ${library_type} ${CMAKE_CURRENT_SOURCE_DIR}/src/${MULTIVERSE_CLIENT}.cpp)
    target_include_directories(${MULTIVERSE_CLIENT_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_directories(${MULTIVERSE_CLIENT_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${MULTIVERSE_CLIENT_LIB} PRIVATE c++abi zmq atomic)
    target_compile_options(${MULTIVERSE_CLIENT_LIB} PRIVATE ${BUILD_FLAGS})
    if(${library_type} STREQUAL STATIC)
        install(
            PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/lib${MULTIVERSE_CLIENT_LIB}.a DESTINATION ${LIB_DIR}/${STDLIB} RENAME lib${MULTIVERSE_CLIENT}.a
        )
    elseif(${library_type} STREQUAL SHARED)
        install(
            PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/lib${MULTIVERSE_CLIENT_LIB}.so DESTINATION ${LIB_DIR}/${STDLIB} RENAME lib${MULTIVERSE_CLIENT}.so
        )
    endif()
endfunction()

function(build_multiverse_client_json)
    set(MULTIVERSE_CLIENT_JSON_LIB ${MULTIVERSE_CLIENT_JSON}_${STDLIB})
    add_library(${MULTIVERSE_CLIENT_JSON_LIB} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/${MULTIVERSE_CLIENT_JSON}.cpp)
    target_include_directories(${MULTIVERSE_CLIENT_JSON_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_directories(${MULTIVERSE_CLIENT_JSON_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${MULTIVERSE_CLIENT_JSON_LIB} PRIVATE ${MULTIVERSE_CLIENT}_${STDLIB}_STATIC jsoncpp)
    target_compile_options(${MULTIVERSE_CLIENT_JSON_LIB} PRIVATE ${BUILD_FLAGS})
    install(
        PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/lib${MULTIVERSE_CLIENT_JSON_LIB}.so DESTINATION ${LIB_DIR}/${STDLIB} RENAME lib${MULTIVERSE_CLIENT_JSON}.so
    )
endfunction()

foreach(STDLIB IN LISTS STDLIBS)
    set(BUILD_FLAGS "-stdlib=${STDLIB}")

    build_multiverse_client(SHARED)

    build_multiverse_client(STATIC)

    build_multiverse_client_json()
endforeach()

find_package(pybind11 REQUIRED)
pybind11_add_module(${MULTIVERSE_CLIENT_PYBIND} MODULE ${CMAKE_CURRENT_SOURCE_DIR}/src/${MULTIVERSE_CLIENT_PYBIND}.cpp)
target_include_directories(${MULTIVERSE_CLIENT_PYBIND} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_directories(${MULTIVERSE_CLIENT_PYBIND} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${MULTIVERSE_CLIENT_PYBIND} PRIVATE ${MULTIVERSE_CLIENT}_libstdc++_STATIC)
install(TARGETS ${MULTIVERSE_CLIENT_PYBIND} DESTINATION ${LIB_DIR}/python)