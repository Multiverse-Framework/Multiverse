@echo off

set "MULTIVERSE_DIR=%~dp0multiverse"

rmdir /s /q "%MULTIVERSE_DIR%\build"
cmake -S "%MULTIVERSE_DIR%" -B "%MULTIVERSE_DIR%\build" -DCMAKE_TOOLCHAIN_FILE"=%MULTIVERSE_DIR%\external\vcpkg\scripts\buildsystems\vcpkg.cmake" -DCMAKE_INSTALL_PREFIX:PATH="%MULTIVERSE_DIR%"
cmake --build "%MULTIVERSE_DIR%\build"

rmdir /s /q "%MULTIVERSE_DIR%\bin"
mkdir "%MULTIVERSE_DIR%\bin"
mklink "%MULTIVERSE_DIR%\bin\multiverse_server.exe" "%MULTIVERSE_DIR%\build\src\multiverse_server\Debug\multiverse_server.exe"

rmdir /s /q "%MULTIVERSE_DIR%\lib"
mkdir "%MULTIVERSE_DIR%\lib"
copy "%MULTIVERSE_DIR%\build\src\multiverse_client\Debug\multiverse_client.dll" "%MULTIVERSE_DIR%\lib"