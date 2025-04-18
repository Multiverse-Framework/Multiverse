@echo off
setlocal EnableDelayedExpansion

@REM Check for administrative permissions
net session >nul 2>&1
if !errorlevel! neq 0 (
    echo This script requires administrative privileges.
    echo Please run as an administrator.
    pause
    exit /b 1
)

for /f "delims=" %%i in ('where python 2^>nul') do (
    set "PYTHON_EXECUTABLE=%%i"
    set "PYTHON_DIR=%%~dpi"
    goto :found
)

echo python.exe not found in PATH.
exit /b 1

:found
echo Found Python: !PYTHON_EXECUTABLE!

for /f "tokens=2 delims= " %%v in ('"!PYTHON_EXECUTABLE!" --version 2^>nul') do (
    set "PYTHON_VERSION=%%v"
)

for /f "tokens=1,2 delims=." %%a in ("!PYTHON_VERSION!") do (
    set "PY_MAJOR=%%a"
    set "PY_MINOR=%%b"
)

set "CURRENT_DIR=%~dp0"

cd %CURRENT_DIR%

set "MULTIVERSE_DIR=%CURRENT_DIR%multiverse"

for /f "delims=" %%i in ('powershell -NoProfile -Command ^
    "[System.Environment]::OSVersion.VersionString"') do set OSVERSION=%%i

for /f "usebackq delims=" %%i in (`powershell -NoProfile -Command ^
    "(Get-CimInstance Win32_OperatingSystem).Caption.Trim()"`) do set "OSCAPTION=%%i"

echo Version: !OSVERSION!

echo !OSCAPTION! | findstr "windows 10" >nul
if !errorlevel! == 0 (
    echo Detected Windows 10: !OSCAPTION!
    if not "!PY_MAJOR!"=="3" if not "!PY_MINOR!"=="8" (
        echo Python version is not 3.8: !PYTHON_VERSION!
        exit /b 1
    )
    goto :next
)

echo !OSCAPTION! | findstr "windows 11" >nul
if !errorlevel! == 0 (
    echo Detected Windows 11: !OSCAPTION!
    if not "!PY_MAJOR!"=="3" if not "!PY_MINOR!"=="12" (
        echo Python version is not 3.12: !PYTHON_VERSION!
        exit /b 1
    )
    goto :next
)

echo !OSCAPTION! | findstr "windows server 2022" >nul
if !errorlevel! == 0 (
    echo Detected Windows Server 2022: !OSCAPTION!
    goto :next
)

echo Unknown or unsupported Windows version: !OSCAPTION!
exit /b 1

:next
if not exist "!PYTHON_EXECUTABLE!" (
    echo Python executable not found: !PYTHON_EXECUTABLE!
    exit /b 1
)
echo Python executable path is: !PYTHON_EXECUTABLE!

!PYTHON_EXECUTABLE! -m ensurepip
!PYTHON_EXECUTABLE! -m pip install -U pip virtualenvwrapper-win pytest
echo Python directory: !PYTHON_DIR!

set "SCRIPT_PATH=!PYTHON_DIR!Scripts\mkvirtualenv.bat"
if not exist "!SCRIPT_PATH!" (
    echo mkvirtualenv.bat not found in Scripts directory: !SCRIPT_PATH!
    exit /b 1
)

set "MKVIRTUALENV_EXECUTABLE=%PYTHON_DIR%\Scripts\mkvirtualenv.bat"
set "PYTHON_EXECUTABLE=%USERPROFILE%\Envs\multiverse\Scripts\python.exe"
if not exist "!PYTHON_EXECUTABLE!" (
    powershell -NoProfile -Command "%MKVIRTUALENV_EXECUTABLE% --system-site-packages multiverse"
    @REM Wait for the virtual environment to be created
    TIMEOUT /T 1
)
if not exist "!PYTHON_EXECUTABLE!" (
    echo Python executable not found in virtual environment: !PYTHON_EXECUTABLE!
    exit /b 1
)

echo Change Python executable to: !PYTHON_EXECUTABLE!
!PYTHON_EXECUTABLE! -m pip install build pyyaml

@REM Install chocolatey (https://chocolatey.org/install)
if not exist "C:\ProgramData\chocolatey" (
    powershell -NoProfile -Command "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))"
)

if not exist "C:\ProgramData\chocolatey" (
    echo Chocolatey installation failed.
    pause
    exit /b 1
)

@REM Install cmake 7zip
set "NEW_PATH=%PATH%;%ALLUSERSPROFILE%\chocolatey\bin"
powershell -Command "if (-not ($env:PATH.Split(';') -contains '%ALLUSERSPROFILE%\chocolatey\bin')) {[Environment]::SetEnvironmentVariable('PATH', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%ALLUSERSPROFILE%\chocolatey\bin', [EnvironmentVariableTarget]::User)}"
set "SET_NEW_PATH=$env:Path = '%NEW_PATH%'; [System.Environment]::SetEnvironmentVariable('Path', $env:Path, [System.EnvironmentVariableTarget]::Process)"
powershell -NoProfile -Command "%SET_NEW_PATH%; choco install -y curl cmake 7zip"

echo !PYTHON_VERSION! | findstr /b "3.8" >nul
if !errorlevel! == 0 (
    @REM Install vcredist2013, vcredist140, openssl, curl
    powershell -NoProfile -Command "%SET_NEW_PATH%; choco install -y vcredist2013 vcredist140; choco install -y openssl --version 1.1.1.2100; choco install -y curl cmake 7zip"

    @REM Download OpenCV
    set "OPENCV_DIR=%MULTIVERSE_DIR%\external\opencv"
    set "OPENCV_ZIP=opencv-3.4.6-vc16.VS2019.zip"
    if not exist "%OPENCV_DIR%" (
        mkdir "%OPENCV_DIR%"
        start powershell -NoProfile -Command "curl -o '%OPENCV_DIR%\%OPENCV_ZIP%' 'https://github.com/ros2/ros2/releases/download/opencv-archives/%OPENCV_ZIP%'; Expand-Archive -Path '%OPENCV_DIR%\%OPENCV_ZIP%' -DestinationPath '%OPENCV_DIR%'; setx /m OpenCV_DIR %OPENCV_DIR%"
    )

    @REM Install dependencies for ROS
    set "ASIO_DIR=%MULTIVERSE_DIR%\external\asio"
    set "ASIO_NUPKG=asio.1.12.1.nupkg"
    if not exist "%ASIO_DIR%" (
        mkdir "%ASIO_DIR%"
        start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%ASIO_DIR%\%ASIO_NUPKG%' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/%ASIO_NUPKG%'; choco install -y -s %ASIO_DIR% asio"
    )

    set "BULLET_DIR=%MULTIVERSE_DIR%\external\bullet"
    set "BULLET_NUPKG=bullet.3.17.nupkg"
    if not exist "%BULLET_DIR%" (
        mkdir "%BULLET_DIR%"
        start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%BULLET_DIR%\%BULLET_NUPKG%' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/%BULLET_NUPKG%'; choco install -y -s %BULLET_DIR% bullet"
    )

    set "CUNIT_DIR=%MULTIVERSE_DIR%\external\cunit"
    set "CUNIT_NUPKG=cunit.2.1.3.nupkg"
    if not exist "%CUNIT_DIR%" (
        mkdir "%CUNIT_DIR%"
        start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%CUNIT_DIR%\%CUNIT_NUPKG%' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/%CUNIT_NUPKG%'; choco install -y -s %CUNIT_DIR% cunit"
    )

    set "EIGEN_DIR=%MULTIVERSE_DIR%\external\eigen"
    set "EIGEN_NUPKG=eigen.3.3.4.nupkg"
    if not exist "%EIGEN_DIR%" (
        mkdir "%EIGEN_DIR%"
        start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%EIGEN_DIR%\%EIGEN_NUPKG%' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/%EIGEN_NUPKG%'; choco install -y -s %EIGEN_DIR% eigen"
    )

    set "TINYXML2_DIR=%MULTIVERSE_DIR%\external\tinyxml2"
    set "TINYXML2_NUPKG=tinyxml2.6.0.0.nupkg"
    if not exist "%TINYXML2_DIR%" (
        mkdir "%TINYXML2_DIR%"
        start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%TINYXML2_DIR%\%TINYXML2_NUPKG%' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/%TINYXML2_NUPKG%'; choco install -y -s %TINYXML2_DIR% tinyxml2"
    )
    !PYTHON_EXECUTABLE! -m pip install setuptools==59.6.0 catkin_pkg cryptography empy importlib-metadata jsonschema lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5 pillow psutil pycairo pydot pyparsing==2.4.7 pytest pyyaml

    set "XMLLINT_DIR=%MULTIVERSE_DIR%\external\xmllint"
    set "LIBXML2_7Z=libxml2-2.9.3-win32-x86_64.7z"
    set "ICONV_7Z=iconv-1.14-win32-x86_64.7z"
    set "ZLIB_7Z=zlib-1.2.8-win32-x86_64.7z"
    if not exist "%XMLLINT_DIR%" (
        mkdir "%XMLLINT_DIR%"
        start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%XMLLINT_DIR%\%LIBXML2_7Z%' 'https://www.zlatkovic.com/pub/libxml/64bit/%LIBXML2_7Z%'; 7z x '%XMLLINT_DIR%\%LIBXML2_7Z%' -o'%XMLLINT_DIR%'; pause"
        start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%XMLLINT_DIR%\%ICONV_7Z%' 'https://www.zlatkovic.com/pub/libxml/64bit/%ICONV_7Z%'; 7z x '%XMLLINT_DIR%\%ICONV_7Z%' -o'%XMLLINT_DIR%'; pause"
        start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%XMLLINT_DIR%\%ZLIB_7Z%' 'https://www.zlatkovic.com/pub/libxml/64bit/%ZLIB_7Z%'; 7z x '%XMLLINT_DIR%\%ZLIB_7Z%' -o'%XMLLINT_DIR%'; pause"
        powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%XMLLINT_DIR%\bin', [EnvironmentVariableTarget]::User)"
    )

    set "GRAPHVIZ_DIR=%MULTIVERSE_DIR%\external\graphviz"
    set "GRAPHVIZ_ZIP=windows_10_cmake_Release_Graphviz-12.0.0-win64.zip"
    if not exist "%GRAPHVIZ_DIR%" (
        mkdir "%GRAPHVIZ_DIR%"
        start powershell -NoProfile -Command "curl -o '%GRAPHVIZ_DIR%\%GRAPHVIZ_ZIP%' 'https://gitlab.com/api/v4/projects/4207231/packages/generic/graphviz-releases/12.0.0/%GRAPHVIZ_ZIP%'; Expand-Archive -Path '%GRAPHVIZ_DIR%\%GRAPHVIZ_ZIP%' -DestinationPath '%GRAPHVIZ_DIR%'"
        powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%GRAPHVIZ_DIR%\Graphviz-12.0.0-win64\bin', [EnvironmentVariableTarget]::User)"
    )

    set "ROS_DIR=%MULTIVERSE_DIR%\external\ros\ros2_jazzy"
    set "ROS_ZIP=ros2-jazzy-20240705-windows-release-amd64.zip"
    if not exist "%ROS_DIR%" (
        mkdir "%ROS_DIR%"
        start powershell -NoProfile -Command "curl -o '%ROS_DIR%\%ROS_ZIP%' 'https://github.com/ros2/ros2/releases/download/release-jazzy-20240705/%ROS_ZIP%'; 7z x '%ROS_DIR%\%ROS_ZIP%' -o'%ROS_DIR%'; Move-Item -Path '%ROS_DIR%\ros2-windows\*' '%ROS_DIR%' -Force; Remove-Item -Path '%ROS_DIR%\%ROS_ZIP%'; Remove-Item -Path '%ROS_DIR%\ros2-windows'; workon multiverse; !PYTHON_EXECUTABLE! %ROS_DIR%/../fix_shebang.py %ROS_DIR%"
    )

    @REM Install ros_dep_tools
    !PYTHON_EXECUTABLE! -m pip install rosdep colcon-core
)

@REM Install additional packages for multiverse_parser
@REM !PYTHON_EXECUTABLE! -m pip install -r %MULTIVERSE_DIR%\modules\multiverse_parser\requirements.txt

@REM Install additional packages for multiverse_knowledge
@REM !PYTHON_EXECUTABLE! -m pip install -r %MULTIVERSE_DIR%\modules\multiverse_knowledge\requirements.txt

@REM Install additional packages for mujoco_connector
!PYTHON_EXECUTABLE! -m pip install -r %MULTIVERSE_DIR%\modules\multiverse_connectors\src\multiverse_simulators\src\mujoco_connector\requirements.txt

@REM Install msys2
set "MSYS2_DIR=%MULTIVERSE_DIR%\external\msys2"
if not exist "%MSYS2_DIR%" (
    mkdir "%MSYS2_DIR%"
    powershell -NoProfile -Command "curl -o '%MSYS2_DIR%\msys2-x86_64-20241208.exe' 'https://github.com/msys2/msys2-installer/releases/download/2024-12-08/msys2-x86_64-20241208.exe'; %MSYS2_DIR%\msys2-x86_64-20241208.exe in --confirm-command --accept-messages --root %MSYS2_DIR%"
    powershell -NoProfile -Command "%MSYS2_DIR%\msys2_shell.cmd -defterm -here -no-start -c 'pacman -Syu --noconfirm'"
    powershell -NoProfile -Command "%MSYS2_DIR%\msys2_shell.cmd -defterm -here -no-start -c 'pacman -Sy --noconfirm mingw-w64-x86_64-gcc mingw-w64-x86_64-make mingw-w64-x86_64-cmake mingw-w64-x86_64-zeromq mingw-w64-x86_64-cppzmq mingw-w64-x86_64-jsoncpp mingw-w64-x86_64-boost mingw-w64-x86_64-glfw mingw-w64-x86_64-tinyxml2 mingw-w64-x86_64-eigen3'"
    powershell -NoProfile -Command "%MSYS2_DIR%\msys2_shell.cmd -defterm -here -no-start -c 'pacman -Syu --noconfirm'"
    powershell -NoProfile -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%MULTIVERSE_DIR%\external\msys2\mingw64\bin', [EnvironmentVariableTarget]::User)"
)

echo Installation completed.
pause