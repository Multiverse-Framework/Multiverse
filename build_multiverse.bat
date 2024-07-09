@echo off

set "CONFIG=Release"
set "MULTIVERSE_DIR=%~dp0multiverse"

cmake -S "%MULTIVERSE_DIR%" -B "%MULTIVERSE_DIR%\build" -DCMAKE_TOOLCHAIN_FILE"=%MULTIVERSE_DIR%\external\vcpkg\scripts\buildsystems\vcpkg.cmake" -DCMAKE_INSTALL_PREFIX:PATH="%MULTIVERSE_DIR%" -Wno-deprecated -DBUILD_MODULES=OFF
cmake --build "%MULTIVERSE_DIR%\build" --config %CONFIG% --target INSTALL

// Build the server
g++ -o "%MULTIVERSE_DIR%\bin\multiverse_server.exe" "%MULTIVERSE_DIR%\src\multiverse_server\src\main.cpp" "%MULTIVERSE_DIR%\src\multiverse_server\src\multiverse_server.cpp" -I"%MULTIVERSE_DIR%\src\multiverse_server\include" -lzmq -ljsoncpp

if "%CONFIG%"=="Debug" (
    copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\debug\bin\*.dll" "%MULTIVERSE_DIR%\bin"
) else if "%CONFIG%"=="Release" (
    copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\bin\*.dll" "%MULTIVERSE_DIR%\bin"
)