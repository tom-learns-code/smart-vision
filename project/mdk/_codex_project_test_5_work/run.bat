@echo off
cd /d "%~dp0"

set "PROJECT_PYTHON=%~dp0.venv\Scripts\python.exe"
if not exist "%PROJECT_PYTHON%" (
    echo Project virtual environment not found: .venv
    echo Please configure the environment before running the project.
    pause
    exit /b 1
)

echo Running game...
"%PROJECT_PYTHON%" main.py
pause
