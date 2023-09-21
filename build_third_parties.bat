@echo off

set "MULTIVERSE_DIR=%~dp0multiverse"

if not exist "%MULTIVERSE_DIR%\external\vcpkg" (
    git clone https://github.com/Microsoft/vcpkg.git "%MULTIVERSE_DIR%\external\vcpkg"
    cd "%MULTIVERSE_DIR%\external\vcpkg"
    bootstrap-vcpkg.bat
    vcpkg integrate install
)