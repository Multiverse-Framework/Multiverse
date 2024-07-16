@echo off

@REM Check for administrative permissions
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo This script requires administrative privileges.
    echo Please run as an administrator.
    pause
    exit /b 1
)

set "CURRENT_DIR=%~dp0"

cd %CURRENT_DIR%

set "MULTIVERSE_DIR=%CURRENT_DIR%multiverse"

@REM Install vckpg
if not exist "%MULTIVERSE_DIR%\external\vcpkg" (
    start powershell -NoProfile -Command "git clone https://github.com/Microsoft/vcpkg.git %MULTIVERSE_DIR%\external\vcpkg; cd %MULTIVERSE_DIR%\external\vcpkg; bootstrap-vcpkg.bat; vcpkg integrate install"
)

@REM Install chocolatey (https://chocolatey.org/install)
if not exist "C:\ProgramData\chocolatey" (
    powershell -NoProfile -Command "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))"
)

@REM Install python 3.8.3, vcredist2013, vcredist140, openssl, curl, cmake, mingw
set "NEW_PATH=%PATH%;%ALLUSERSPROFILE%\chocolatey\bin"
set "SET_NEW_PATH=$env:Path = '%NEW_PATH%'; [System.Environment]::SetEnvironmentVariable('Path', $env:Path, [System.EnvironmentVariableTarget]::Process)"
powershell -NoProfile -Command "%SET_NEW_PATH%; choco install -y vcredist2013 vcredist140; choco install -y python --version 3.8.3; choco install -y openssl --version 1.1.1.2100; choco install -y curl cmake mingw"

@REM Download OpenCV
set "OPENCV_DIR=%MULTIVERSE_DIR%\external\opencv"
if not exist "%OPENCV_DIR%" (
    mkdir "%OPENCV_DIR%"
    start powershell -NoProfile -Command "curl -o '%OPENCV_DIR%\opencv-3.4.6-vc16.VS2019.zip' 'https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip'; Expand-Archive -Path '%OPENCV_DIR%\opencv-3.4.6-vc16.VS2019.zip' -DestinationPath '%OPENCV_DIR%'; setx /m OpenCV_DIR %OPENCV_DIR%"
)

@REM Install dependencies for ROS
set "ASIO_DIR=%MULTIVERSE_DIR%\external\asio"
if not exist "%ASIO_DIR%" (
    mkdir "%ASIO_DIR%"
    start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%ASIO_DIR%\asio.1.12.1.nupkg' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/asio.1.12.1.nupkg'; choco install -y -s %ASIO_DIR% asio"
)

set "BULLET_DIR=%MULTIVERSE_DIR%\external\bullet"
if not exist "%BULLET_DIR%" (
    mkdir "%BULLET_DIR%"
    start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%BULLET_DIR%\bullet.3.17.nupkg' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/bullet.3.17.nupkg'; choco install -y -s %BULLET_DIR% bullet"
)

set "CUNIT_DIR=%MULTIVERSE_DIR%\external\cunit"
if not exist "%CUNIT_DIR%" (
    mkdir "%CUNIT_DIR%"
    start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%CUNIT_DIR%\cunit.2.1.3.nupkg' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/cunit.2.1.3.nupkg'; choco install -y -s %CUNIT_DIR% cunit"
)

set "EIGEN_DIR=%MULTIVERSE_DIR%\external\eigen"
if not exist "%EIGEN_DIR%" (
    mkdir "%EIGEN_DIR%"
    start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%EIGEN_DIR%\eigen.3.3.4.nupkg' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/eigen.3.3.4.nupkg'; choco install -y -s %EIGEN_DIR% eigen"
)

set "TINYXML_DIR=%MULTIVERSE_DIR%\external\tinyxml"
if not exist "%TINYXML_DIR%" (
    mkdir "%TINYXML_DIR%"
    start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%TINYXML_DIR%\tinyxml2.6.0.0.nupkg' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/tinyxml2.6.0.0.nupkg'; choco install -y -s %TINYXML_DIR% tinyxml2"
)

set "PYTHON_EXECUTABLE=C:\Python38\python.exe"
%PYTHON_EXECUTABLE% -m pip install -U pip virtualenvwrapper-win
set "MKVIRTUALENV_EXECUTABLE=C:\Python38\Scripts\mkvirtualenv.bat"
set "PYTHON_EXECUTABLE=%USERPROFILE%\Envs\multiverse\Scripts\python.exe"
if not exist "%PYTHON_EXECUTABLE%" (
    powershell -NoProfile -Command "%MKVIRTUALENV_EXECUTABLE% --system-site-packages multiverse"
    @REM Wait for the virtual environment to be created
    TIMEOUT /T 1
)
%PYTHON_EXECUTABLE% -m pip install -U pip 
%PYTHON_EXECUTABLE% -m pip install setuptools==59.6.0 catkin_pkg cryptography empy importlib-metadata jsonschema lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5 pillow psutil pycairo pydot pyparsing==2.4.7 pytest pyyaml

set "XMLLINT_DIR=%MULTIVERSE_DIR%\external\xmllint"
if not exist "%XMLLINT_DIR%" (
    mkdir "%XMLLINT_DIR%"
    start powershell -NoProfile -Command "curl -o '%XMLLINT_DIR%\libxml2-2.9.3-win32-x86_64.7z' 'https://www.zlatkovic.com/pub/libxml/64bit/libxml2-2.9.3-win32-x86_64.7z'; 7z x '%XMLLINT_DIR%\libxml2-2.9.3-win32-x86_64.7z' -o'%XMLLINT_DIR%'"
    start powershell -NoProfile -Command "curl -o '%XMLLINT_DIR%\iconv-1.14-win32-x86_64.7z' 'https://www.zlatkovic.com/pub/libxml/64bit/iconv-1.14-win32-x86_64.7z'; 7z x '%XMLLINT_DIR%\iconv-1.14-win32-x86_64.7z' -o'%XMLLINT_DIR%'"
    start powershell -NoProfile -Command "curl -o '%XMLLINT_DIR%\zlib-1.2.8-win32-x86_64.7z' 'https://www.zlatkovic.com/pub/libxml/64bit/zlib-1.2.8-win32-x86_64.7z'; 7z x '%XMLLINT_DIR%\zlib-1.2.8-win32-x86_64.7z' -o'%XMLLINT_DIR%'"
    powershell -Command "[System.Environment]::SetEnvironmentVariable('Path', $env:Path + ';%XMLLINT_DIR%\bin', [System.EnvironmentVariableTarget]::Machine)"
)

set "GRAPHVIZ_DIR=%MULTIVERSE_DIR%\external\graphviz"
if not exist "%GRAPHVIZ_DIR%" (
    mkdir "%GRAPHVIZ_DIR%"
    start powershell -NoProfile -Command "curl -o '%GRAPHVIZ_DIR%\windows_10_cmake_Release_Graphviz-12.0.0-win64.zip' 'https://gitlab.com/api/v4/projects/4207231/packages/generic/graphviz-releases/12.0.0/windows_10_cmake_Release_Graphviz-12.0.0-win64.zip'; Expand-Archive -Path '%GRAPHVIZ_DIR%\windows_10_cmake_Release_Graphviz-12.0.0-win64.zip' -DestinationPath '%GRAPHVIZ_DIR%'"
    powershell -Command "[System.Environment]::SetEnvironmentVariable('Path', $env:Path + ';%GRAPHVIZ_DIR%\Graphviz-12.0.0-win64\bin', [System.EnvironmentVariableTarget]::Machine)"
)

set "ROS_DIR=%MULTIVERSE_DIR%\external\ros\ros2_jazzy"
if not exist "%ROS_DIR%" (
    mkdir "%ROS_DIR%"
    set "ROS_ZIP=ros2-jazzy-20240705-windows-release-amd64.zip"
    start powershell -NoProfile -Command "curl -o '%ROS_DIR%\%ROS_ZIP%' 'https://github.com/ros2/ros2/releases/download/release-jazzy-20240705/%ROS_ZIP%'; 7z x '%ROS_DIR%\%ROS_ZIP%' -o'%ROS_DIR%';  Remove-Item -Path '%ROS_DIR%\%ROS_ZIP%'; Move-Item -Path '%ROS_DIR%\ros2-windows\*' '%ROS_DIR%'; Remove-Item -Path '%ROS_DIR%\ros2-windows'"
    workon multiverse
    %PYTHON_EXECUTABLE% %ROS_DIR%/../fix_shebang.py %ROS_DIR%
)

@REM Install dependencies for the rest packages

@REM Install build
%PYTHON_EXECUTABLE% -m pip install build

@REM Install additional packages for USD and multiverse_knowledge
%PYTHON_EXECUTABLE% -m pip install pyside6 pyopengl wheel cython owlready2 markupsafe==2.0.1 jinja2 pybind11

@REM Install mujoco
%PYTHON_EXECUTABLE% -m pip install mujoco==3.1.5

@REM Install urdf_parser_py
%PYTHON_EXECUTABLE% -m pip install urdf_parser_py

echo Installation completed.
pause