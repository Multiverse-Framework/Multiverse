@echo off

set "MULTIVERSE_DIR=%~dp0multiverse"

cmake -S "%MULTIVERSE_DIR%" -B "%MULTIVERSE_DIR%\build" -DCMAKE_TOOLCHAIN_FILE"=%MULTIVERSE_DIR%\external\vcpkg\scripts\buildsystems\vcpkg.cmake" -DCMAKE_INSTALL_PREFIX:PATH="%MULTIVERSE_DIR%"
cmake --build "%MULTIVERSE_DIR%\build" --config Debug --target INSTALL

copy /y "%MULTIVERSE_DIR%\build\src\multiverse_server\Debug\jsoncpp.dll" "%MULTIVERSE_DIR%\bin"
copy /y "%MULTIVERSE_DIR%\build\src\multiverse_server\Debug\libzmq-mt-gd-4_3_5.dll" "%MULTIVERSE_DIR%\bin"