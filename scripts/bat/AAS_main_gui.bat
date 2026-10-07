@echo off
rem ── go to the project folder ────────────────────────────────
cd /d "%~dp0..\.."

rem ── activate the virtual environment ───────────────
call ".venv\Scripts\activate.bat" 2>nul || call "venv\Scripts\activate.bat" 2>nul || (
    echo venv not found & pause & exit /b 1
)

:: Ensure required packages are installed
python -m pip show navpy >nul 2>&1 || python -m pip install -e .

set PYTHONPATH=%cd%\src;%PYTHONPATH%

:: Run the app
python -m navpy.main_gui
if errorlevel 1 (
    echo Error occurred! & pause
)
exit
