@echo off

set "CURRENT_DIR=%~dp0"

set "MULTIVERSE_DIR=%CURRENT_DIR%multiverse"

set "ROS_DIR=%MULTIVERSE_DIR%\external\ros"
set "ROS2_DISTRO="
for %%d in (foxy jazzy) do (
    if exist "%ROS_DIR%\ros2_%%d\setup.bat" (
        set "ROS2_DISTRO=%%d"
        goto :found
    )
)

:found
if "%ROS2_DISTRO%"=="" (
    echo No ROS2 distro found
    exit /b 1
)

set "VCVARS64=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VCVARS64%" (
    echo "Visual Studio 2022 not found: %VCVARS64%"
    exit /b 1
) else (
    REM Build the workspace
    cd /d "%~dp0\multiverse_ws2"
    workon multiverse
    call "%VCVARS64%"
    call "%ROS_DIR%\ros2_%ROS2_DISTRO%\setup.bat"
    colcon build
)
