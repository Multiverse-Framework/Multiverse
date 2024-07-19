@echo off

set "CONFIG=Release"
set "MULTIVERSE_DIR=%~dp0multiverse"
set "MSYS2_BIN_DIR=%MULTIVERSE_DIR%\external\msys2\mingw64\bin"
set "CMAKE_EXECUTE=%MSYS2_BIN_DIR%\cmake.exe"
set "NINJA_EXECUTE=%MSYS2_BIN_DIR%\ninja.exe"
set "CMAKE_C_EXECUTE=%MSYS2_BIN_DIR%\gcc.exe"
set "CMAKE_CXX_EXECUTE=%MSYS2_BIN_DIR%\g++.exe"

if exist "%CMAKE_EXECUTE%" if exist "%NINJA_EXECUTE%" (
    workon multiverse
    powershell -NoProfile -Command "%SET_NEW_PATH%; %CMAKE_EXECUTE% -S %MULTIVERSE_DIR% -B %MULTIVERSE_DIR%\build -G Ninja -DCMAKE_MAKE_PROGRAM=%NINJA_EXECUTE% -D CMAKE_C_COMPILER=%CMAKE_C_EXECUTE% -D CMAKE_CXX_COMPILER=%CMAKE_CXX_EXECUTE% -DCMAKE_INSTALL_PREFIX:PATH=%MULTIVERSE_DIR% -Wno-deprecated -Wno-dev -DBUILD_MODULES=ON"
    powershell -NoProfile -Command "%SET_NEW_PATH%; %CMAKE_EXECUTE% --build %MULTIVERSE_DIR%\build --config %CONFIG%"
    powershell -NoProfile -Command "%SET_NEW_PATH%; %CMAKE_EXECUTE% --install %MULTIVERSE_DIR%\build"
    
    copy /Y "%MSYS2_BIN_DIR%\libstdc++-6.dll" "%MULTIVERSE_DIR%\lib\dist-packages"
    copy /Y "%MSYS2_BIN_DIR%\libwinpthread-1.dll" "%MULTIVERSE_DIR%\lib\dist-packages"
    copy /Y "%MSYS2_BIN_DIR%\libzmq.dll" "%MULTIVERSE_DIR%\lib\dist-packages"
    copy /Y "%MSYS2_BIN_DIR%\libgcc_s_seh-1.dll" "%MULTIVERSE_DIR%\lib\dist-packages"
    copy /Y "%MSYS2_BIN_DIR%\libsodium-26.dll" "%MULTIVERSE_DIR%\lib\dist-packages"

    powershell -Command "if (-not ($env:PATH.Split(';') -contains '%MULTIVERSE_DIR%\bin')) {[Environment]::SetEnvironmentVariable('PATH', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%MULTIVERSE_DIR%\bin', [EnvironmentVariableTarget]::User)}"
    powershell -Command "if (-not ($env:PYTHONPATH.Split(';') -contains '%MULTIVERSE_DIR%\lib\dist-packages')) {[Environment]::SetEnvironmentVariable('PYTHONPATH', [Environment]::GetEnvironmentVariable('PYTHONPATH', 'User') + ';%MULTIVERSE_DIR%\lib\dist-packages', [EnvironmentVariableTarget]::User)}"
    pause
) else (
    echo "%CMAKE_EXECUTE% not found. Please run install.bat first."
    pause
)