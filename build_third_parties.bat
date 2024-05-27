@echo off

set "CURRENT_DIR=%~dp0"

git submodule update --init

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
    @REM Create the folder if it doesn't exist
    mkdir "%BLENDER_BUILD_DIR%"
    echo "Folder created: %BLENDER_BUILD_DIR%"
) else (
    echo "Folder already exists: %BLENDER_BUILD_DIR%"
)

@REM cd "%BLENDER_EXT_DIR%\blender" && make update
cd "%BLENDER_EXT_DIR%\blender" && cmake -S . -B "..\..\..\build\blender" && cmake --build "..\..\..\build\blender" --target INSTALL --config Release
@REM cd "%BLENDER_EXT_DIR%\lib\win64_vc15\python\310\bin" && python.exe -m pip install --upgrade pip build --no-warn-script-location && python.exe -m pip install bpy --no-warn-script-location

@REM @REM Build USD

@REM set "USD_BUILD_DIR=%BUILD_DIR%\USD"
@REM set "USD_EXT_DIR=%EXT_DIR%\USD"
@REM if not exist "%USD_BUILD_DIR%" (
@REM     @REM Create the folder if it doesn't exist
@REM     mkdir "%USD_BUILD_DIR%"
@REM     echo "Folder created: %USD_BUILD_DIR%"
@REM ) else (
@REM     echo "Folder already exists: %USD_BUILD_DIR%"
@REM )

@REM @REM python %USD_EXT_DIR%\build_scripts\build_usd.py %USD_BUILD_DIR%

@REM @REM Build MuJoCo

@REM set "MUJOCO_BUILD_DIR=%BUILD_DIR%\mujoco"
@REM set "MUJOCO_EXT_DIR=%EXT_DIR%\mujoco"
@REM if not exist "%MUJOCO_BUILD_DIR%" (
@REM     @REM Create the folder if it doesn't exist
@REM     mkdir "%MUJOCO_BUILD_DIR%"
@REM     echo "Folder created: %MUJOCO_BUILD_DIR%"
@REM ) else (
@REM     echo "Folder already exists: %MUJOCO_BUILD_DIR%"
@REM )

@REM cd %MUJOCO_BUILD_DIR% && cmake %MUJOCO_EXT_DIR% -DCMAKE_INSTALL_PREFIX=%MUJOCO_BUILD_DIR% -Wno-deprecated -Wno-dev && cmake --build . --config Release && cmake --install . && cd "%CURRENT_DIR%"
@REM copy /y "%MUJOCO_BUILD_DIR%\bin\mujoco.dll" "%MULTIVERSE_DIR%\bin"