@echo off

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

@REM Install python 3.8.3, vcredist2013, vcredist140, openssl, curl, cmake
set "NEW_PATH=%PATH%;%ALLUSERSPROFILE%\chocolatey\bin"
set "SET_NEW_PATH=$env:Path = '%NEW_PATH%'; [System.Environment]::SetEnvironmentVariable('Path', $env:Path, [System.EnvironmentVariableTarget]::Process)"
powershell -NoProfile -Command "%SET_NEW_PATH%; choco install -y vcredist2013 vcredist140; choco install -y python --version 3.8.3; choco install -y openssl --version 1.1.1.2100; choco install -y curl cmake; pause"

@REM Download OpenCV
set "OPENCV_DIR=%MULTIVERSE_DIR%\external\opencv"
if not exist "%OPENCV_DIR%" (
    mkdir "%OPENCV_DIR%"
    start powershell -NoProfile -Command "curl -o '%OPENCV_DIR%\opencv-3.4.6-vc16.VS2019.zip' 'https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip'; Expand-Archive -Path '%OPENCV_DIR%\opencv-3.4.6-vc16.VS2019.zip' -DestinationPath '%OPENCV_DIR%\..'; setx /m OpenCV_DIR %OPENCV_DIR%"
)

@REM Install dependencies
set "ASIO_DIR=%MULTIVERSE_DIR%\external\asio"
if not exist "%ASIO_DIR%" (
    mkdir "%ASIO_DIR%"
    start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%ASIO_DIR%\asio.1.12.1.nupkg' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/asio.1.12.1.nupkg'; choco install -y -s %ASIO_DIR% asio; pause"
)

set "BULLET_DIR=%MULTIVERSE_DIR%\external\bullet"
if not exist "%BULLET_DIR%" (
    mkdir "%BULLET_DIR%"
    start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%BULLET_DIR%\bullet.3.17.nupkg' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/bullet.3.17.nupkg'; choco install -y -s %BULLET_DIR% bullet; pause"
)

set "CUNIT_DIR=%MULTIVERSE_DIR%\external\cunit"
if not exist "%CUNIT_DIR%" (
    mkdir "%CUNIT_DIR%"
    start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%CUNIT_DIR%\cunit.2.1.3.nupkg' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/cunit.2.1.3.nupkg'; choco install -y -s %CUNIT_DIR% cunit; pause"
)

set "EIGEN_DIR=%MULTIVERSE_DIR%\external\eigen"
if not exist "%EIGEN_DIR%" (
    mkdir "%EIGEN_DIR%"
    start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%EIGEN_DIR%\eigen.3.3.4.nupkg' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/eigen.3.3.4.nupkg'; choco install -y -s %EIGEN_DIR% eigen; pause"
)

set "TINYXML_DIR=%MULTIVERSE_DIR%\external\tinyxml"
if not exist "%TINYXML_DIR%" (
    mkdir "%TINYXML_DIR%"
    start powershell -NoProfile -Command "%SET_NEW_PATH%; curl -o '%TINYXML_DIR%\tinyxml2.6.0.0.nupkg' 'https://github.com/ros2/choco-packages/releases/download/2022-03-15/tinyxml2.6.0.0.nupkg'; choco install -y -s %TINYXML_DIR% tinyxml2; pause"
)

set "PYTHON_EXECUTABLE=C:\Python38\python.exe"
%PYTHON_EXECUTABLE% -m pip install -U pip virtualenvwrapper-win
set "MKVIRTUALENV_EXECUTABLE=C:\Python38\Scripts\mkvirtualenv.bat"
powershell -NoProfile -Command "%MKVIRTUALENV_EXECUTABLE% --system-site-packages multiverse"
set "PYTHON_EXECUTABLE=%USERPROFILE%\Envs\multiverse\Scripts\python.exe"
%PYTHON_EXECUTABLE% -m pip install -U catkin_pkg cryptography empy importlib-metadata jsonschema lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5 pillow psutil pycairo pydot pyparsing==2.4.7 pytest pyyaml
pause

@REM @REM Upgrade pip
@REM python -m pip install --upgrade pip build

@REM @REM Install additional packages for USD and multiverse_knowledge
@REM python -m pip install pyside6 pyopengl wheel cython owlready2 markupsafe==2.0.1 jinja2 pybind11

@REM @REM Install mujoco
@REM python -m pip install mujoco==3.1.5

@REM @REM Install pyyaml
@REM python -m pip install pyyaml

@REM @REM Install urdf_parser_py
@REM python -m pip install urdf_parser_py