@echo off
setlocal EnableDelayedExpansion

REM Read first argument (e.g., enable_pause)
set "ENABLE_PAUSE=%1"

REM Check if it matches "enable_pause"
if "!ENABLE_PAUSE!"=="enable_pause" (
    echo Pause is enabled
    set "SHOULD_PAUSE=1"
) else (
    echo Pause not enabled
    set "SHOULD_PAUSE=0"
)

set "PYTHON_EXECUTABLE=%USERPROFILE%\Envs\multiverse\Scripts\python.exe"
if not exist "%PYTHON_EXECUTABLE%" (
    echo Python executable not found: %PYTHON_EXECUTABLE%
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

@REM Build CMake
set "CMAKE_BUILD_DIR=%BUILD_DIR%\CMake"
set "CMAKE_FILE_NAME=cmake-4.0.0-windows-x86_64"
set "CMAKE_ZIP_FILE=%CMAKE_FILE_NAME%.zip"
if not exist "%CMAKE_BUILD_DIR%" (
    powershell -NoProfile -Command "curl -o '%EXT_DIR%\%CMAKE_ZIP_FILE%' 'https://github.com/Kitware/CMake/releases/download/v4.0.0/%CMAKE_ZIP_FILE%'"
    powershell -NoProfile -Command "7z x '%EXT_DIR%\%CMAKE_ZIP_FILE%' -o'%BUILD_DIR%'"
    powershell -NoProfile -Command "move '%BUILD_DIR%\%CMAKE_FILE_NAME%' '%CMAKE_BUILD_DIR%'"
)
set "CMAKE_EXECUTABLE=%CMAKE_BUILD_DIR%\bin\cmake.exe"
if not exist "%CMAKE_EXECUTABLE%" (
    for /f "delims=" %%i in ('where cmake') do (
        set "CMAKE_EXECUTABLE=%%i"
        goto :found
    )

    :found
    if not exist "%CMAKE_EXECUTABLE%" (
        echo CMake executable not found
        exit /b 1
    )
)
echo Using CMake executable: %CMAKE_EXECUTABLE%
if "!SHOULD_PAUSE!"=="1" (
    pause
)

@REM Build blender

set "BLENDER_BUILD_DIR=%BUILD_DIR%\blender"
set "BLENDER_EXT_DIR=%EXT_DIR%\blender-git"
set "BLENDER_FILE_NAME=blender-4.4.0-windows-x64"
set "BLENDER_ZIP_FILE=%BLENDER_FILE_NAME%.zip"

set "FROM_SRC=0"
if not exist "%BLENDER_BUILD_DIR%" (
    @REM Check if FROM_SRC is set to 1
    if "%FROM_SRC%"=="1" (
        git submodule update --init "%BLENDER_EXT_DIR%/blender"

        @REM Create the folder if it doesn't exist
        mkdir "%BLENDER_BUILD_DIR%"
        echo Folder created: %BLENDER_BUILD_DIR%

        powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%BLENDER_BUILD_DIR%\bin\Release', [EnvironmentVariableTarget]::User)"
        powershell -NoProfile -Command "cd '%BLENDER_EXT_DIR%\blender'; .\make update"
        powershell -NoProfile -Command "cd '%BLENDER_EXT_DIR%\blender'; %CMAKE_EXECUTABLE% -S . -B '..\..\..\build\blender'; %CMAKE_EXECUTABLE% --build '..\..\..\build\blender' --target INSTALL --config Release"
        powershell -NoProfile -Command "cd '%BLENDER_EXT_DIR%\blender\lib\windows_x64\python\311\bin'; .\python.exe -m pip install --upgrade pip build --no-warn-script-location; .\python.exe -m pip install bpy --no-warn-script-location"
    ) else (
        powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%BLENDER_BUILD_DIR%', [EnvironmentVariableTarget]::User)"
        powershell -NoProfile -Command "curl -o '%EXT_DIR%\%BLENDER_ZIP_FILE%' 'https://download.blender.org/release/Blender4.4/%BLENDER_ZIP_FILE%'"
        powershell -NoProfile -Command "7z x '%EXT_DIR%\%BLENDER_ZIP_FILE%' -o'%BUILD_DIR%'"
        powershell -NoProfile -Command "move '%BUILD_DIR%\%BLENDER_FILE_NAME%' '%BLENDER_BUILD_DIR%'"
        powershell -NoProfile -Command "cd '%BLENDER_BUILD_DIR%\4.4\python\bin'; .\python.exe -m pip install --upgrade pip build --no-warn-script-location; .\python.exe -m pip install bpy --no-warn-script-location"
    )
) else (
    echo Folder already exists: %BLENDER_BUILD_DIR%
)
if "!SHOULD_PAUSE!"=="1" (
    pause
)

@REM Build MuJoCo

set "MUJOCO_BUILD_DIR=%BUILD_DIR%\mujoco"
set "MUJOCO_EXT_DIR=%EXT_DIR%\mujoco"
set "MUJOCO_VERSION=3.3.0"
set "MUJOCO_FILE_NAME=mujoco-%MUJOCO_VERSION%-windows-x86_64"
set "MUJOCO_ZIP_FILE=%MUJOCO_FILE_NAME%.zip"

set "FROM_SRC=1"
if not exist "%MUJOCO_BUILD_DIR%" (
    @REM Check if FROM_SRC is set to 1
    if "%FROM_SRC%"=="1" (
        git submodule update --init "%MUJOCO_EXT_DIR%"

        @REM Create the folder if it doesn't exist
        mkdir "%MUJOCO_BUILD_DIR%"
        echo Folder created: %MUJOCO_BUILD_DIR%

        powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%MUJOCO_BUILD_DIR%\bin', [EnvironmentVariableTarget]::User)"
        powershell -NoProfile -Command "cd %MUJOCO_BUILD_DIR%; %CMAKE_EXECUTABLE% %MUJOCO_EXT_DIR% -DCMAKE_INSTALL_PREFIX=%MUJOCO_BUILD_DIR% -DCMAKE_POLICY_VERSION_MINIMUM='3.5' -Wno-deprecated -Wno-dev;  %CMAKE_EXECUTABLE% --build . --config Release -- /p:VcpkgEnableManifest=true;  %CMAKE_EXECUTABLE% --install ."
    ) else (
        powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%MUJOCO_BUILD_DIR%\bin', [EnvironmentVariableTarget]::User)"
        powershell -NoProfile -Command "curl -o '%EXT_DIR%\%MUJOCO_ZIP_FILE%' 'https://github.com/google-deepmind/mujoco/releases/download/%MUJOCO_VERSION%/%MUJOCO_ZIP_FILE%'"
        powershell -NoProfile -Command "7z x '%EXT_DIR%\%MUJOCO_ZIP_FILE%' -o'%MUJOCO_BUILD_DIR%'"
    )
) else (
    echo Folder already exists: %MUJOCO_BUILD_DIR%
)
if "!SHOULD_PAUSE!"=="1" (
    pause
)

@REM Build pybind11

set "PYBIND11_BUILD_DIR=%BUILD_DIR%\pybind11"
set "PYBIND11_EXT_DIR=%EXT_DIR%\pybind11"
if not exist "%PYBIND11_BUILD_DIR%" (
    git submodule update --init "%PYBIND11_EXT_DIR%"

    @REM Create the folder if it doesn't exist
    mkdir "%PYBIND11_BUILD_DIR%"
    echo Folder created: %PYBIND11_BUILD_DIR%

    powershell -NoProfile -Command "cd %PYBIND11_BUILD_DIR%; %CMAKE_EXECUTABLE% -G 'MinGW Makefiles' %PYBIND11_EXT_DIR% -DCMAKE_INSTALL_PREFIX=%PYBIND11_BUILD_DIR% -DDOWNLOAD_CATCH=ON -DDOWNLOAD_EIGEN=ON -Wno-deprecated -Wno-dev; %CMAKE_EXECUTABLE% --build .; %CMAKE_EXECUTABLE% --install ."
) else (
    echo Folder already exists: %PYBIND11_BUILD_DIR%
)
if "!SHOULD_PAUSE!"=="1" (
    pause
)

@REM @REM Build USD

@REM set "USD_BUILD_DIR=%BUILD_DIR%\USD"
@REM set "USD_EXT_DIR=%EXT_DIR%\USD"
@REM set "VCVARS64=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
@REM if not exist "%USD_BUILD_DIR%" (
@REM     git submodule update --init "%USD_EXT_DIR%"

@REM     @REM Create the folder if it doesn't exist
@REM     mkdir "%USD_BUILD_DIR%"
@REM     echo Folder created: %USD_BUILD_DIR%

@REM     if not exist "%VCVARS64%" (
@REM         echo Visual Studio 2022 not found: %VCVARS64%
@REM         exit /b 1
@REM     )
@REM     powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%USD_BUILD_DIR%\bin;%USD_BUILD_DIR%\lib', [EnvironmentVariableTarget]::User)"
@REM     powershell -Command "[Environment]::SetEnvironmentVariable('PYTHONPATH', [Environment]::GetEnvironmentVariable('PYTHONPATH', 'User') + ';%USD_BUILD_DIR%\lib\python', [EnvironmentVariableTarget]::User)"
@REM     workon multiverse
@REM     call "%VCVARS64%"
@REM     powershell -NoProfile -Command "%PYTHON_EXECUTABLE% %USD_EXT_DIR%\build_scripts\build_usd.py %USD_BUILD_DIR%"
@REM ) else (
@REM     echo Folder already exists: %USD_BUILD_DIR%
@REM )

echo Third parties built successfully
if "!SHOULD_PAUSE!"=="1" (
    pause
)