if (UNIX)
    execute_process(
        COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/mujoco_compile.py ${BIN_DIR}/mujoco_compile
    )
elseif (WIN32)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/scripts/mujoco_compile.py ${BIN_DIR}/mujoco_compile.py
    )
endif()

set(mujoco_DIR ${BUILD_DIR}/mujoco/lib/cmake/mujoco)
find_package(mujoco REQUIRED)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(tinyxml2 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
    ${BUILD_DIR}/mujoco/include
    ${SRC_DIR}/multiverse_client/include
    ${Boost_INCLUDE_DIRS}
)

add_executable(mujoco_headless ${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco_headless.cpp)
add_library(mujoco_headless_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mj_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mj_multiverse_client.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mj_simulate.cpp
)

if(UNIX)
    target_link_libraries(mujoco_headless
        mujoco_headless_lib
        multiverse_client_json
        Threads::Threads
        jsoncpp
        boost_filesystem
        ${BUILD_DIR}/mujoco/lib/libmujoco.so
    )
    set_target_properties(mujoco_headless PROPERTIES INSTALL_RPATH "${BUILD_DIR}/mujoco/lib;${LIB_DIR}/libstdc++")
elseif(WIN32)
    target_link_libraries(mujoco_headless PRIVATE
        mujoco_headless_lib
        multiverse_client_json
        Threads::Threads
        Boost::filesystem
        tinyxml2::tinyxml2
        mujoco::mujoco
    )
endif()

install(TARGETS mujoco_headless DESTINATION ${BIN_DIR})

add_executable(mujoco ${CMAKE_CURRENT_SOURCE_DIR}/src/mujoco.cpp)
add_library(mujoco_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mj_visual.cpp
)

if(UNIX)
    target_link_libraries(mujoco
        mujoco_lib
        mujoco_headless_lib
        multiverse_client_json
        glfw
        GL
        Threads::Threads
        jsoncpp
        boost_filesystem
        ${BUILD_DIR}/mujoco/lib/libmujoco.so
    )
    set_target_properties(mujoco PROPERTIES INSTALL_RPATH "${BUILD_DIR}/mujoco/lib;${LIB_DIR}/libstdc++")
elseif(WIN32)
    target_link_libraries(mujoco PRIVATE
        mujoco_lib
        mujoco_headless_lib
        multiverse_client_json
        glfw
        OpenGL::GL
        tinyxml2::tinyxml2
        Boost::filesystem
        mujoco::mujoco
    )
endif()

install(TARGETS mujoco DESTINATION ${BIN_DIR})