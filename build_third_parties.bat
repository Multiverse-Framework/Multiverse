@echo off

set "PYTHON_EXECUTABLE=%USERPROFILE%\Envs\multiverse\Scripts\python.exe"
if not exist "%PYTHON_EXECUTABLE%" (
    echo "Python executable not found: %PYTHON_EXECUTABLE%"
    exit /b 1
)

set "CURRENT_DIR=%~dp0"

cd %CURRENT_DIR%

set "MULTIVERSE_DIR=%CURRENT_DIR%multiverse"

set "BIN_DIR=%MULTIVERSE_DIR%\bin"
if not exist "%BIN_DIR%" (
    @REM Create the folder if it doesn't exist
    mkdir "%BIN_DIR%"
)

set "EXT_DIR=%MULTIVERSE_DIR%\external"

set "BUILD_DIR=%MULTIVERSE_DIR%\build"

set "SRC_DIR=%MULTIVERSE_DIR%\src"

set "INCLUDE_DIR=%MULTIVERSE_DIR%\include"

@REM Build blender

set "BLENDER_BUILD_DIR=%BUILD_DIR%\blender"
set "BLENDER_EXT_DIR=%EXT_DIR%\blender-git"

if not exist "%BLENDER_BUILD_DIR%" (
    git submodule update --init "%BLENDER_EXT_DIR%/blender"

    @REM Create the folder if it doesn't exist
    mkdir "%BLENDER_BUILD_DIR%"
    echo "Folder created: %BLENDER_BUILD_DIR%"

    powershell -NoProfile -Command "cd '%BLENDER_EXT_DIR%\blender'; .\make update"
    powershell -NoProfile -Command "cd '%BLENDER_EXT_DIR%\blender'; cmake -S . -B '..\..\..\build\blender'; cmake --build '..\..\..\build\blender' --target INSTALL --config Release"
    powershell -NoProfile -Command "cd '%BLENDER_EXT_DIR%\blender\lib\windows_x64\python\311\bin'; .\python.exe -m pip install --upgrade pip build --no-warn-script-location; .\python.exe -m pip install bpy --no-warn-script-location"
    powershell -Command "[System.Environment]::SetEnvironmentVariable('Path', $env:Path + ';%BLENDER_BUILD_DIR%\bin\Release', [System.EnvironmentVariableTarget]::Machine)"
) else (
    echo "Folder already exists: %BLENDER_BUILD_DIR%"
)

@REM Build USD

set "USD_BUILD_DIR=%BUILD_DIR%\USD"
set "USD_EXT_DIR=%EXT_DIR%\USD"
set "VCVARS64=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if exist "%USD_BUILD_DIR%" (
    git submodule update --init "%USD_EXT_DIR%"

    @REM Create the folder if it doesn't exist
    @REM mkdir "%USD_BUILD_DIR%"
    echo "Folder created: %USD_BUILD_DIR%"

    if not exist "%VCVARS64%" (
        echo "Visual Studio 2022 not found: %VCVARS64%"
        exit /b 1
    )
    workon multiverse
    call "%VCVARS64%"
    powershell -NoProfile -Command "%PYTHON_EXECUTABLE% %USD_EXT_DIR%\build_scripts\build_usd.py %USD_BUILD_DIR%"
    powershell -Command "[System.Environment]::SetEnvironmentVariable('Path', $env:Path + ';%USD_BUILD_DIR%\bin;%USD_BUILD_DIR%\lib', [System.EnvironmentVariableTarget]::Machine)"
    powershell -Command "[System.Environment]::SetEnvironmentVariable('PYTHONPATH', $env:PYTHONPATH + ';%USD_BUILD_DIR%\lib\python', [System.EnvironmentVariableTarget]::Machine)"
) else (
    echo "Folder already exists: %USD_BUILD_DIR%"
)

@REM Build MuJoCo

set "MUJOCO_BUILD_DIR=%BUILD_DIR%\mujoco"
set "MUJOCO_EXT_DIR=%EXT_DIR%\mujoco"
if not exist "%MUJOCO_BUILD_DIR%" (
    git submodule update --init "%MUJOCO_EXT_DIR%"

    @REM Create the folder if it doesn't exist
    mkdir "%MUJOCO_BUILD_DIR%"
    echo "Folder created: %MUJOCO_BUILD_DIR%"

    powershell -NoProfile -Command "cd %MUJOCO_BUILD_DIR%; cmake %MUJOCO_EXT_DIR% -DCMAKE_INSTALL_PREFIX=%MUJOCO_BUILD_DIR% -Wno-deprecated -Wno-dev; cmake --build . --config Release; cmake --install ."
    powershell -Command "[System.Environment]::SetEnvironmentVariable('Path', $env:Path + ';%MUJOCO_BUILD_DIR%\bin', [System.EnvironmentVariableTarget]::Machine)"
) else (
    echo "Folder already exists: %MUJOCO_BUILD_DIR%"
)

echo "Third parties built successfully"
pause