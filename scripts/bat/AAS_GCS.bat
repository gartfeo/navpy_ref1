@echo off
rem ── go to the project folder ────────────────────────────────
cd /d "%~dp0..\.."

rem ── activate the virtual environment ───────────────
call ".venv\Scripts\activate.bat" 2>nul || call "venv\Scripts\activate.bat" 2>nul || (
    echo venv not found & pause & exit /b 1
)

:: Ensure backend dependencies are installed
python -m pip show uvicorn >nul 2>&1 || python -m pip install -r src\gcs\requirements.txt

:: Launch backend + frontend + SITL on this directory's slot: the slot this
:: directory already owns is reused, otherwise a free one is reserved, so
:: stacks from different directories don't collide. Args pass through to
:: gcs_launch.py, e.g.  AAS_GCS.bat --no-sitl
python scripts\gcs_launch.py %*

exit /b %errorlevel%
