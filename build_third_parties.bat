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

if not exist "%BLENDER_EXT_DIR%\lib" (
    cd %BLENDER_EXT_DIR% && mkdir lib && cd lib && svn checkout https://svn.blender.org/svnroot/bf-blender/trunk/lib/win64_vc15 && cd %CURRENT_DIR%
    cd %BLENDER_EXT_DIR%\blender && make update && cd %CURRENT_DIR%
)

cd "%BLENDER_EXT_DIR%\blender" && cmake -S . -B "../../../build/blender" && cmake --build "../../../build/blender" && cd "%CURRENT_DIR%"
@REM cd "%BLENDER_BUILD_DIR%\bin\3.6\python\pip"
@REM python3.10 -m pip install --upgrade pip build --no-warn-script-location