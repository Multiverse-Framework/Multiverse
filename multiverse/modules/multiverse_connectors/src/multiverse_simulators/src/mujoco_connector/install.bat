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

@REM Copy files and directories
xcopy /E /I /Y %CURRENT_DIR%\plugin %MUJOCO_SRC_DIR%\plugin
copy /Y %CURRENT_DIR%\..\..\..\..\..\..\src\multiverse_client\include\multiverse_client.h %MUJOCO_SRC_DIR%\plugin\multiverse_connector
copy /Y %CURRENT_DIR%\..\..\..\..\..\..\src\multiverse_client\include\multiverse_client_json.h %MUJOCO_SRC_DIR%\plugin\multiverse_connector
copy /Y %CURRENT_DIR%\..\..\..\..\..\..\src\multiverse_client\src\multiverse_client.cpp %MUJOCO_SRC_DIR%\plugin\multiverse_connector
copy /Y %CURRENT_DIR%\..\..\..\..\..\..\src\multiverse_client\src\multiverse_client_json.cpp %MUJOCO_SRC_DIR%\plugin\multiverse_connector

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
if exist "%MUJOCO_PLUGIN_DIR%" (
    mkdir "%MUJOCO_PLUGIN_DIR%"
)

@REM Run cmake commands
pushd "%MUJOCO_BUILD_DIR%"
cmake "%MUJOCO_SRC_DIR%" -DCMAKE_INSTALL_PREFIX="%MUJOCO_BUILD_DIR%" -Wno-deprecated -Wno-dev
if errorlevel 1 exit /b 1
cmake --build . 
if errorlevel 1 exit /b 1
cmake --install . 
if errorlevel 1 exit /b 1
copy /Y "%MUJOCO_BUILD_DIR%\lib\libmultiverse_connector.dll" "%MUJOCO_PLUGIN_DIR%"
popd
