@echo off

@REM Check if MUJOCO_SRC_DIR or MUJOCO_BUILD_DIR is set
if "%MUJOCO_SRC_DIR%"=="" (
    echo MUJOCO_SRC_DIR is unset.
    exit /b 1
)
if "%MUJOCO_BUILD_DIR%"=="" (
    echo MUJOCO_BUILD_DIR is unset.
    exit /b 1
)

set "MUJOCO_SRC_DIR=%MUJOCO_SRC_DIR:/=\%"
echo MUJOCO_SRC_DIR is set to: %MUJOCO_SRC_DIR%
echo MUJOCO_BUILD_DIR is set to: %MUJOCO_BUILD_DIR%
set CURRENT_DIR=%CD%
set MULTIVERSE_DIR=%CURRENT_DIR%\..\..\..\..\..\..

@REM Copy files and directories
xcopy /E /I /Y %CURRENT_DIR%\plugin %MUJOCO_SRC_DIR%\plugin
mklink /H %MUJOCO_SRC_DIR%\plugin\multiverse_connector\multiverse_client.h %MULTIVERSE_DIR%\src\multiverse_client\include\multiverse_client.h
mklink /H %MUJOCO_SRC_DIR%\plugin\multiverse_connector\multiverse_client_json.h %MULTIVERSE_DIR%\src\multiverse_client\include\multiverse_client_json.h
mklink /H %MUJOCO_SRC_DIR%\plugin\multiverse_connector\multiverse_client.cpp %MULTIVERSE_DIR%\src\multiverse_client\src\multiverse_client.cpp
mklink /H %MUJOCO_SRC_DIR%\plugin\multiverse_connector\multiverse_client_json.cpp %MULTIVERSE_DIR%\src\multiverse_client\src\multiverse_client_json.cpp

@REM Specify the file path
set MUJOCO_CMAKE_PATH=%MUJOCO_SRC_DIR%\CMakeLists.txt

@REM Specify the line to add
set LINE_TO_ADD=add_subdirectory(plugin/multiverse_connector^^^)

@REM Check if the line already exists in the file
findstr /C:"plugin/multiverse_connector" "%MUJOCO_CMAKE_PATH%" >nul
if errorlevel 1 (
    echo %LINE_TO_ADD% >> "%MUJOCO_CMAKE_PATH%"
)

@REM Create plugin directory
set MUJOCO_PLUGIN_DIR=%MUJOCO_BUILD_DIR%\bin\mujoco_plugin
if not exist "%MUJOCO_PLUGIN_DIR%" (
    mkdir "%MUJOCO_PLUGIN_DIR%"
)

set "CMAKE_BIN_DIR=C:\Program Files\CMake\bin"

@REM Run cmake commands
pushd "%MUJOCO_BUILD_DIR%"
"%CMAKE_BIN_DIR%\cmake.exe" "%MUJOCO_SRC_DIR%" -DCMAKE_INSTALL_PREFIX="%MUJOCO_BUILD_DIR%" -Wno-deprecated -Wno-dev
if errorlevel 1 exit /b 1
"%CMAKE_BIN_DIR%\cmake.exe" --build . --config Release -- /p:VcpkgEnableManifest=true
if errorlevel 1 exit /b 1
"%CMAKE_BIN_DIR%\cmake.exe" --install . 
if errorlevel 1 exit /b 1
copy /Y "%MUJOCO_BUILD_DIR%\bin\Release\multiverse_connector.dll" "%MUJOCO_PLUGIN_DIR%"
popd
