@echo off

taskkill /IM multiverse_server.exe /F

set "MULTIVERSE_PATH=%~dp0.."

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

%PYTHON_EXECUTABLE% %MULTIVERSE_PATH%\modules\multiverse_connectors\scripts\launch_multiverse_server.py --muv_file=%MUV_FILE%
%PYTHON_EXECUTABLE% %MULTIVERSE_PATH%\modules\multiverse_connectors\scripts\launch_simulators.py --muv_file=%MUV_FILE%

echo [multiverse_launch] Running... Press Ctrl+C to exit
:loop
timeout /t 1 >nul
goto loop
