@echo off

set "CONFIG=Release"
set "MULTIVERSE_DIR=%~dp0multiverse"

cmake -S "%MULTIVERSE_DIR%" -B "%MULTIVERSE_DIR%\build" -DCMAKE_TOOLCHAIN_FILE"=%MULTIVERSE_DIR%\external\vcpkg\scripts\buildsystems\vcpkg.cmake" -DCMAKE_INSTALL_PREFIX:PATH="%MULTIVERSE_DIR%" -Wno-deprecated
cmake --build "%MULTIVERSE_DIR%\build" --config %CONFIG% --target INSTALL

if "%CONFIG%"=="Debug" (
    copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\debug\bin\*.dll" "%MULTIVERSE_DIR%\bin"
) else if "%CONFIG%"=="Release" (
    copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\bin\*.dll" "%MULTIVERSE_DIR%\bin"
)