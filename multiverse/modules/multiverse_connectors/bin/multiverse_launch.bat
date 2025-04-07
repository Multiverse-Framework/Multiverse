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

@REM set "ROS_DIR=%MULTIVERSE_DIR%\external\ros"
@REM set "ROS2_DISTRO="
@REM for %%d in (foxy jazzy) do (
@REM     if exist "%ROS_DIR%\ros2_%%d\setup.bat" (
@REM         set "ROS2_DISTRO=%%d"
@REM         goto :found
@REM     )
@REM )

@REM :found
@REM if "%ROS2_DISTRO%"=="" (
@REM     echo No ROS2 distro found
@REM ) else (
@REM     cd /d "%~dp0\multiverse_ws2"
@REM     workon multiverse
@REM     call "%VCVARS64%"
@REM     call "%ROS_DIR%\ros2_%ROS2_DISTRO%\setup.bat"
@REM     call "%MULTIVERSE_DIR%\..\multiverse_ws2\setup.bat"
@REM     %PYTHON_EXECUTABLE% %MULTIVERSE_DIR%\modules\multiverse_connectors\scripts\launch_ros.py --muv_file=%MUV_FILE%
@REM )

echo [multiverse_launch] Running... Press Ctrl+C to exit
:loop
timeout /t 1 >nul
goto loop
