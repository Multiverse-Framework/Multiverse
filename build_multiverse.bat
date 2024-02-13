@echo off

set "CONFIG=Release"
set "MULTIVERSE_DIR=%~dp0multiverse"

cmake -S "%MULTIVERSE_DIR%" -B "%MULTIVERSE_DIR%\build" -DCMAKE_TOOLCHAIN_FILE"=%MULTIVERSE_DIR%\external\vcpkg\scripts\buildsystems\vcpkg.cmake" -DCMAKE_INSTALL_PREFIX:PATH="%MULTIVERSE_DIR%" -Wno-deprecated
cmake --build "%MULTIVERSE_DIR%\build" --config %CONFIG% --target INSTALL

copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\bin\boost_atomic-vc143-mt-x64-1_83.dll" "%MULTIVERSE_DIR%\bin"
copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\bin\boost_filesystem-vc143-mt-x64-1_83.dll" "%MULTIVERSE_DIR%\bin"
copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\bin\boost_system-vc143-mt-x64-1_83.dll" "%MULTIVERSE_DIR%\bin"
copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\bin\jsoncpp.dll" "%MULTIVERSE_DIR%\bin"
copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\bin\libzmq-mt-4_3_5.dll" "%MULTIVERSE_DIR%\bin"
copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\bin\glfw3.dll" "%MULTIVERSE_DIR%\bin"
copy /y "%MULTIVERSE_DIR%\build\vcpkg_installed\x64-windows\bin\tinyxml2.dll" "%MULTIVERSE_DIR%\bin"