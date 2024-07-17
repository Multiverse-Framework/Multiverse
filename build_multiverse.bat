@echo off

set "CONFIG=Release"
set "MULTIVERSE_DIR=%~dp0multiverse"
set "CMAKE_EXECUTE=%MULTIVERSE_DIR%\external\msys2\mingw64\bin\cmake.exe"

if exist "%CMAKE_EXECUTE%" (
    workon multiverse
    powershell -NoProfile -Command "%SET_NEW_PATH%; %CMAKE_EXECUTE% -S '%MULTIVERSE_DIR%' -B '%MULTIVERSE_DIR%\build' -G Ninja -DCMAKE_MAKE_PROGRAM=%MULTIVERSE_DIR%\external\msys2\mingw64\bin\ninja.exe -D CMAKE_C_COMPILER=%MULTIVERSE_DIR%\external\msys2\mingw64\bin\gcc.exe -D CMAKE_CXX_COMPILER='%MULTIVERSE_DIR%\external\msys2\mingw64\bin\g++.exe' -DCMAKE_INSTALL_PREFIX:PATH='%MULTIVERSE_DIR%' -Wno-deprecated -Wno-dev -DBUILD_MODULES=ON"
    powershell -NoProfile -Command "%SET_NEW_PATH%; %CMAKE_EXECUTE% --build '%MULTIVERSE_DIR%\build' --config %CONFIG%"
    powershell -NoProfile -Command "%SET_NEW_PATH%; %CMAKE_EXECUTE% --install '%MULTIVERSE_DIR%\build'"
    powershell -Command "if (-not ($env:PATH.Split(';') -contains '%MULTIVERSE_DIR%\bin')) {[Environment]::SetEnvironmentVariable('PATH', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%MULTIVERSE_DIR%\bin', [EnvironmentVariableTarget]::User)}"

    pause
) else (
    echo "%CMAKE_EXECUTE% not found. Please run install.bat first."
    pause
)