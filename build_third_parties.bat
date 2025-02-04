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
set "BLENDER_FILE_NAME=blender-4.2.0-windows-x64"
set "BLENDER_ZIP_FILE=%BLENDER_FILE_NAME%.zip"

set "FROM_SRC=0"
if not exist "%BLENDER_BUILD_DIR%" (
    @REM Check if FROM_SRC is set to 1
    if "%FROM_SRC%"=="1" (
        git submodule update --init "%BLENDER_EXT_DIR%/blender"

        @REM Create the folder if it doesn't exist
        mkdir "%BLENDER_BUILD_DIR%"
        echo "Folder created: %BLENDER_BUILD_DIR%"

        powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%BLENDER_BUILD_DIR%\bin\Release', [EnvironmentVariableTarget]::User)"
        powershell -NoProfile -Command "cd '%BLENDER_EXT_DIR%\blender'; .\make update"
        powershell -NoProfile -Command "cd '%BLENDER_EXT_DIR%\blender'; cmake -S . -B '..\..\..\build\blender'; cmake --build '..\..\..\build\blender' --target INSTALL --config Release"
        powershell -NoProfile -Command "cd '%BLENDER_EXT_DIR%\blender\lib\windows_x64\python\311\bin'; .\python.exe -m pip install --upgrade pip build --no-warn-script-location; .\python.exe -m pip install bpy --no-warn-script-location"
    ) else (
        powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%BLENDER_BUILD_DIR%', [EnvironmentVariableTarget]::User)"
        powershell -NoProfile -Command "curl -o '%EXT_DIR%\%BLENDER_ZIP_FILE%' 'https://download.blender.org/release/Blender4.2/%BLENDER_ZIP_FILE%'"
        powershell -NoProfile -Command "7z x '%EXT_DIR%\%BLENDER_ZIP_FILE%' -o'%BUILD_DIR%'"
        powershell -NoProfile -Command "move '%BUILD_DIR%\%BLENDER_FILE_NAME%' '%BLENDER_BUILD_DIR%'"
        powershell -NoProfile -Command "cd '%BLENDER_BUILD_DIR%\4.2\python\bin'; .\python.exe -m pip install --upgrade pip build --no-warn-script-location; .\python.exe -m pip install bpy --no-warn-script-location"
    )
) else (
    echo "Folder already exists: %BLENDER_BUILD_DIR%"
)

pause

@REM Build MuJoCo

set "MUJOCO_BUILD_DIR=%BUILD_DIR%\mujoco"
set "MUJOCO_EXT_DIR=%EXT_DIR%\mujoco"
set "MUJOCO_VERSION=3.2.6"
set "MUJOCO_FILE_NAME=mujoco-%MUJOCO_VERSION%-windows-x86_64"
set "MUJOCO_ZIP_FILE=%MUJOCO_FILE_NAME%.zip"

set "FROM_SRC=1"
if not exist "%MUJOCO_BUILD_DIR%" (
    @REM Check if FROM_SRC is set to 1
    if "%FROM_SRC%"=="1" (
        
        git submodule update --init "%MUJOCO_EXT_DIR%"

        @REM Create the folder if it doesn't exist
        mkdir "%MUJOCO_BUILD_DIR%"
        echo "Folder created: %MUJOCO_BUILD_DIR%"

        powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%MUJOCO_BUILD_DIR%\bin', [EnvironmentVariableTarget]::User)"
        powershell -NoProfile -Command "cd %MUJOCO_BUILD_DIR%; cmake %MUJOCO_EXT_DIR% -DCMAKE_INSTALL_PREFIX=%MUJOCO_BUILD_DIR% -Wno-deprecated -Wno-dev; cmake --build . --config Release -- /p:VcpkgEnableManifest=true; cmake --install ."
    ) else (
        powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%MUJOCO_BUILD_DIR%\bin', [EnvironmentVariableTarget]::User)"
        powershell -NoProfile -Command "curl -o '%EXT_DIR%\%MUJOCO_ZIP_FILE%' 'https://github.com/google-deepmind/mujoco/releases/download/%MUJOCO_VERSION%/%MUJOCO_ZIP_FILE%'"
        powershell -NoProfile -Command "7z x '%EXT_DIR%\%MUJOCO_ZIP_FILE%' -o'%MUJOCO_BUILD_DIR%'"
    )
) else (
    echo "Folder already exists: %MUJOCO_BUILD_DIR%"
)

pause

@REM Build pybind11

set "PYBIND11_BUILD_DIR=%BUILD_DIR%\pybind11"
set "PYBIND11_EXT_DIR=%EXT_DIR%\pybind11"
if not exist "%PYBIND11_BUILD_DIR%" (
    git submodule update --init "%PYBIND11_EXT_DIR%"

    @REM Create the folder if it doesn't exist
    mkdir "%PYBIND11_BUILD_DIR%"
    echo "Folder created: %PYBIND11_BUILD_DIR%"

    powershell -NoProfile -Command "cd %PYBIND11_BUILD_DIR%; cmake %PYBIND11_EXT_DIR% -DCMAKE_INSTALL_PREFIX=%PYBIND11_BUILD_DIR% -Wno-deprecated -Wno-dev; cmake --build .; cmake --install ."
) else (
    echo "Folder already exists: %PYBIND11_BUILD_DIR%"
)

pause

@REM Build USD

set "USD_BUILD_DIR=%BUILD_DIR%\USD"
set "USD_EXT_DIR=%EXT_DIR%\USD"
set "VCVARS64=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%USD_BUILD_DIR%" (
    git submodule update --init "%USD_EXT_DIR%"

    @REM Create the folder if it doesn't exist
    mkdir "%USD_BUILD_DIR%"
    echo "Folder created: %USD_BUILD_DIR%"

    if not exist "%VCVARS64%" (
        echo "Visual Studio 2022 not found: %VCVARS64%"
        exit /b 1
    )
    powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%USD_BUILD_DIR%\bin;%USD_BUILD_DIR%\lib', [EnvironmentVariableTarget]::User)"
    powershell -Command "[Environment]::SetEnvironmentVariable('PYTHONPATH', [Environment]::GetEnvironmentVariable('PYTHONPATH', 'User') + ';%USD_BUILD_DIR%\lib\python', [EnvironmentVariableTarget]::User)"
    workon multiverse
    call "%VCVARS64%"
    powershell -NoProfile -Command "%PYTHON_EXECUTABLE% %USD_EXT_DIR%\build_scripts\build_usd.py %USD_BUILD_DIR%"
) else (
    echo "Folder already exists: %USD_BUILD_DIR%"
)

echo "Third parties built successfully"
pause