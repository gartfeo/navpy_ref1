@echo off
rem ── go to the project folder ────────────────────────────────
cd /d "%~dp0..\.."

rem ── activate the virtual environment ───────────────
call ".venv\Scripts\activate.bat" 2>nul || call "venv\Scripts\activate.bat" 2>nul || (
    echo venv not found & pause & exit /b 1
)

:: Ensure backend dependencies are installed
python -m pip show uvicorn >nul 2>&1 || python -m pip install -r src\gcs\requirements.txt

:: Allocate a free per-chat port set and launch backend + frontend.
:: Each run picks the lowest free chat index so multiple stacks don't collide.
:: Pass-through args, e.g.  AAS_GCS.bat --chat 2
python scripts\gcs_launch.py %*

exit /b %errorlevel%
