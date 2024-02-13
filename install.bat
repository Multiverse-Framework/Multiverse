@echo off

set "CURRENT_DIR=%~dp0"

cd %CURRENT_DIR%

set "MULTIVERSE_DIR=%CURRENT_DIR%multiverse"

@REM Install vckpg

if not exist "%MULTIVERSE_DIR%\external\vcpkg" (
    git clone https://github.com/Microsoft/vcpkg.git "%MULTIVERSE_DIR%\external\vcpkg"
    cd "%MULTIVERSE_DIR%\external\vcpkg"
    bootstrap-vcpkg.bat
    vcpkg integrate install
)

@REM Upgrade pip
python -m pip install --upgrade pip build

@REM Install additional packages for USD and multiverse_knowledge
python -m pip install pyside6 pyopengl wheel cython owlready2 markupsafe==2.0.1 jinja2 pybind11

@REM Install mujoco
python -m pip install mujoco==3.1.0

@REM Install pyyaml
python -m pip install pyyaml

@REM Install urdf_parser_py
python -m pip install urdf_parser_py