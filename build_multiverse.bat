@echo off

set "CMAKE_TOOL_CHAIN=%1"

set "MULTIVERSE_DIR=%~dp0multiverse"
set "CONFIG=Release"
set "MSYS2_BIN_DIR=%MULTIVERSE_DIR%\external\msys2\mingw64\bin"
set "CMAKE_BIN_DIR=C:\Program Files\CMake\bin"
set "PYTHON_EXECUTABLE=%USERPROFILE%\Envs\multiverse\Scripts\python.exe"
if not exist "%PYTHON_EXECUTABLE%" (
    echo "Python executable not found: %PYTHON_EXECUTABLE%"
    exit /b 1
)

if exist "%MULTIVERSE_DIR%\build\CMakeCache.txt" del /F "%MULTIVERSE_DIR%\build\CMakeCache.txt"

set "BUILD_WITH_VCPKG=1"
set "BUILD_WITH_MSYS2=1"

if "%CMAKE_TOOL_CHAIN%"=="vcpkg" (
    set "BUILD_WITH_MSYS2=0"
) else if "%CMAKE_TOOL_CHAIN%"=="msys2" (
    set "BUILD_WITH_VCPKG=0"
)

if "%BUILD_WITH_VCPKG%"=="1" (
    echo Building with vcpkg

    if not exist "%MULTIVERSE_DIR%\external\vcpkg" (
        git clone https://github.com/Microsoft/vcpkg.git "%MULTIVERSE_DIR%\external\vcpkg"
        cd "%MULTIVERSE_DIR%\external\vcpkg"
        bootstrap-vcpkg.bat
        vcpkg integrate install
    )

    if exist "%CMAKE_BIN_DIR%\cmake.exe" (
        powershell -NoProfile -Command "workon multiverse; & '%CMAKE_BIN_DIR%\cmake.exe' -S %MULTIVERSE_DIR% -B %MULTIVERSE_DIR%\build -DCMAKE_TOOLCHAIN_FILE=%MULTIVERSE_DIR%\external\vcpkg\scripts\buildsystems\vcpkg.cmake -DCMAKE_INSTALL_PREFIX:PATH=%MULTIVERSE_DIR% -Wno-deprecated -DBUILD_MODULES=OFF -DPYTHON_EXECUTABLE=%PYTHON_EXECUTABLE%"
        powershell -NoProfile -Command "workon multiverse; & '%CMAKE_BIN_DIR%\cmake.exe' --build %MULTIVERSE_DIR%\build --config %CONFIG% --target INSTALL"

        powershell -Command "if (-not ($env:PATH.Split(';') -contains '%MULTIVERSE_DIR%\bin')) {[Environment]::SetEnvironmentVariable('PATH', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%MULTIVERSE_DIR%\bin', [EnvironmentVariableTarget]::User)}"
        powershell -Command "if (-not ($env:PYTHONPATH.Split(';') -contains '%MULTIVERSE_DIR%\lib\dist-packages')) {[Environment]::SetEnvironmentVariable('PYTHONPATH', [Environment]::GetEnvironmentVariable('PYTHONPATH', 'User') + ';%MULTIVERSE_DIR%\lib\dist-packages', [EnvironmentVariableTarget]::User)}"
        pause
    ) else (
        echo "%CMAKE_BIN_DIR%\cmake.exe not found. Please run install.bat first."
        pause
    )
)

if "%CMAKE_TOOL_CHAIN%"=="" (
    del /F "%MULTIVERSE_DIR%\build\CMakeCache.txt"
    rmdir /S /Q "%MULTIVERSE_DIR%\build\CMakeFiles"
)

if "%BUILD_WITH_MSYS2%"=="1" (
    echo Building with msys2

    if exist "%MSYS2_BIN_DIR%\cmake.exe" if exist "%MSYS2_BIN_DIR%\ninja.exe" (
        powershell -NoProfile -Command "workon multiverse; %MSYS2_BIN_DIR%\cmake.exe -S %MULTIVERSE_DIR% -B %MULTIVERSE_DIR%\build -G Ninja -DCMAKE_MAKE_PROGRAM=%MSYS2_BIN_DIR%\ninja.exe -D CMAKE_C_COMPILER=%MSYS2_BIN_DIR%\gcc.exe -D CMAKE_CXX_COMPILER=%MSYS2_BIN_DIR%\g++.exe -DCMAKE_INSTALL_PREFIX:PATH=%MULTIVERSE_DIR% -Wno-deprecated -Wno-dev -DBUILD_MODULES=ON -DBUILD_TESTS=OFF -DPYTHON_EXECUTABLE=%PYTHON_EXECUTABLE%"
        powershell -NoProfile -Command "workon multiverse; %MSYS2_BIN_DIR%\cmake.exe --build %MULTIVERSE_DIR%\build --config %CONFIG%"
        powershell -NoProfile -Command "workon multiverse; %MSYS2_BIN_DIR%\cmake.exe --install %MULTIVERSE_DIR%\build"
        
        copy /Y "%MSYS2_BIN_DIR%\libstdc++-6.dll" "%MULTIVERSE_DIR%\lib\dist-packages"
        copy /Y "%MSYS2_BIN_DIR%\libwinpthread-1.dll" "%MULTIVERSE_DIR%\lib\dist-packages"
        copy /Y "%MSYS2_BIN_DIR%\libzmq.dll" "%MULTIVERSE_DIR%\lib\dist-packages"
        copy /Y "%MSYS2_BIN_DIR%\libgcc_s_seh-1.dll" "%MULTIVERSE_DIR%\lib\dist-packages"
        copy /Y "%MSYS2_BIN_DIR%\libsodium-26.dll" "%MULTIVERSE_DIR%\lib\dist-packages"

        powershell -Command "if (-not ($env:PATH.Split(';') -contains '%MULTIVERSE_DIR%\bin')) {[Environment]::SetEnvironmentVariable('PATH', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%MULTIVERSE_DIR%\bin', [EnvironmentVariableTarget]::User)}"
        powershell -Command "if (-not ($env:PYTHONPATH.Split(';') -contains '%MULTIVERSE_DIR%\lib\dist-packages')) {[Environment]::SetEnvironmentVariable('PYTHONPATH', [Environment]::GetEnvironmentVariable('PYTHONPATH', 'User') + ';%MULTIVERSE_DIR%\lib\dist-packages', [EnvironmentVariableTarget]::User)}"
        pause
    ) else (
        echo "%MSYS2_BIN_DIR%\cmake.exe not found. Please run install.bat first."
        pause
    )
)