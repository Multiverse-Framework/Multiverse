@echo off

taskkill /IM multiverse_server.exe /F

set "MULTIVERSE_DIR=%~dp0.."

REM Check if an argument is provided
if "%~1"=="" (
    echo Usage: %0 path\to\your\project.muv"
    exit /b 1
)

set "MUV_FILE=%~1"
set "PYTHON_EXECUTABLE=%USERPROFILE%\Envs\multiverse\Scripts\python.exe"
if not exist %MUV_FILE% (
    echo Error: File %MUV_FILE% not found.
    exit /b 1
)

%PYTHON_EXECUTABLE% %MULTIVERSE_DIR%\modules\multiverse_connectors\scripts\launch_multiverse_server.py --muv_file=%MUV_FILE%
%PYTHON_EXECUTABLE% %MULTIVERSE_DIR%\modules\multiverse_connectors\scripts\launch_simulators.py --muv_file=%MUV_FILE%
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
) else (
    cd /d "%~dp0\multiverse_ws2"
    workon multiverse
    call "%VCVARS64%"
    call "%ROS_DIR%\ros2_%ROS2_DISTRO%\setup.bat"
    call "%MULTIVERSE_DIR%\..\multiverse_ws2\setup.bat"
    %PYTHON_EXECUTABLE% %MULTIVERSE_DIR%\modules\multiverse_connectors\scripts\launch_ros.py --muv_file=%MUV_FILE%
)

echo [multiverse_launch] Running... Press Ctrl+C to exit
:loop
timeout /t 1 >nul
goto loop
