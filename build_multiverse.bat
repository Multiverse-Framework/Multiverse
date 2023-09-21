@echo off

set "CURRENT_DIR=%~dp0"

rmdir /s /q %CURRENT_DIR%multiverse\build
cmake -S %CURRENT_DIR%multiverse -B %CURRENT_DIR%multiverse\build -DCMAKE_TOOLCHAIN_FILE=%CURRENT_DIR%multiverse\external\vcpkg\scripts\buildsystems\vcpkg.cmake -DCMAKE_INSTALL_PREFIX:PATH=%CURRENT_DIR%multiverse -DMULTIVERSE_CLIENT_LIBRARY_TYPE=SHARED -DSTDLIB=libc++
cmake --build %CURRENT_DIR%multiverse\build

rmdir /s /q %CURRENT_DIR%multiverse\bin
mkdir %CURRENT_DIR%multiverse\bin
mklink %CURRENT_DIR%multiverse\bin\multiverse_server.exe %CURRENT_DIR%multiverse\build\src\multiverse_server\Debug\multiverse_server.exe