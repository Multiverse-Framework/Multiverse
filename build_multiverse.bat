@echo off

set "CONFIG=Release"
set "MULTIVERSE_DIR=%~dp0multiverse"
set "CMAKE_EXECUTE=C:\Multiverse\multiverse\external\msys2\mingw64\bin\cmake.exe"
"%CMAKE_EXECUTE%" -S "%MULTIVERSE_DIR%" -B "%MULTIVERSE_DIR%\build" -D CMAKE_C_COMPILER=C:/Multiverse/multiverse/external/msys2/mingw64/bin/gcc.exe -D CMAKE_CXX_COMPILER=C:/Multiverse/multiverse/external/msys2/mingw64/bin/g++.exe -DCMAKE_INSTALL_PREFIX:PATH="%MULTIVERSE_DIR%" -Wno-deprecated -Wno-dev -DBUILD_MODULES=ON
"%CMAKE_EXECUTE%" --build "%MULTIVERSE_DIR%\build" --config %CONFIG%
"%CMAKE_EXECUTE%" --install "%MULTIVERSE_DIR%\build"

powershell -Command "$PATHS = [Environment]::GetEnvironmentVariable('PATH'); if (-not ($PATHS.Split(';') -contains '%MULTIVERSE_DIR%\bin')) {[Environment]::SetEnvironmentVariable('PATH', $env:Path + ';%MULTIVERSE_DIR%\bin', [EnvironmentVariableTarget]::User)}"

pause