@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "SCRIPT_NAME=%~nx0"
set "CHILD_MODE=0"
set "DRY_RUN=0"
set "CLEAN_ONLY=0"

:PARSE_ARGS
if "%~1"=="" goto :ARGS_DONE
if /i "%~1"=="__CHILD" (
  set "CHILD_MODE=1"
  shift /1
  goto :PARSE_ARGS
)
if /i "%~1"=="--dry-run" (
  set "DRY_RUN=1"
  shift /1
  goto :PARSE_ARGS
)
if /i "%~1"=="--clean-only" (
  set "CLEAN_ONLY=1"
  shift /1
  goto :PARSE_ARGS
)
if /i "%~1"=="--help" goto :USAGE
if /i "%~1"=="/?" goto :USAGE
if not defined WSL_DISTRO (
  set "WSL_DISTRO=%~1"
) else (
  echo ERROR: Unexpected argument: %~1
  echo(
  goto :USAGE
)
shift /1
goto :PARSE_ARGS

:ARGS_DONE
where wsl.exe >nul 2>&1
if errorlevel 1 (
  echo ERROR: wsl.exe was not found. Install Windows Subsystem for Linux first.
  echo(
  pause
  exit /b 1
)

if not defined WSL_DISTRO call :DETECT_DEFAULT_DISTRO
if not defined WSL_DISTRO (
  echo ERROR: Could not detect the default WSL distro.
  echo Pass it explicitly, for example:
  echo   %SCRIPT_NAME% Ubuntu-24.04
  echo(
  pause
  exit /b 1
)

call :FIND_VHDX
if not defined VHDX (
  echo ERROR: Could not find ext4.vhdx for "%WSL_DISTRO%".
  echo Refusing to guess on a multi-distro system.
  echo(
  pause
  exit /b 1
)

echo =========================================================
echo WSL disk optimizer
echo =========================================================
echo Distro : %WSL_DISTRO%
echo Home   : WSL default user's $HOME
echo VHDX   : %VHDX%
echo(

if "%DRY_RUN%"=="1" (
  echo Dry run only. No files were deleted and no compaction was run.
  exit /b 0
)

if "%CHILD_MODE%"=="1" goto :CHILD
goto :PARENT

:USAGE
echo Usage:
echo   %SCRIPT_NAME% [DistroName] [--dry-run] [--clean-only]
echo(
echo Examples:
echo   %SCRIPT_NAME%
echo   %SCRIPT_NAME% Ubuntu-24.04
echo   %SCRIPT_NAME% Ubuntu-24.04 --dry-run
echo   %SCRIPT_NAME% Ubuntu-24.04 --clean-only
echo(
exit /b 0

:PARENT
echo =========================================================
echo Step A: Clean ArduPilot logs inside WSL
echo =========================================================
call :CLEAN_WSL_LOGS
set "LOG_CLEAN_ERR=%ERRORLEVEL%"
echo(

echo =========================================================
echo Step B: Trim free blocks inside WSL
echo =========================================================
call :RUN_FSTRIM
set "FSTRIM_ERR=%ERRORLEVEL%"
echo(

if not "%LOG_CLEAN_ERR%"=="0" (
  echo WARNING: WSL log cleanup reported exit code %LOG_CLEAN_ERR%.
)
if not "%FSTRIM_ERR%"=="0" (
  echo WARNING: fstrim reported exit code %FSTRIM_ERR%.
)

if "%CLEAN_ONLY%"=="1" (
  echo Clean-only mode finished. Compaction was skipped.
  echo(
  exit /b 0
)

echo =========================================================
echo Step C: Compact the VHDX from an elevated window
echo =========================================================
call :IS_ADMIN
if not errorlevel 1 (
  echo Already elevated; compacting in this window.
  echo(
  goto :CHILD
)

echo Requesting administrator rights for DiskPart...
powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -FilePath '%~f0' -ArgumentList @('__CHILD','%WSL_DISTRO%') -Verb RunAs"
if errorlevel 1 (
  echo ERROR: Could not start elevated compaction window.
  echo(
  pause
  exit /b 1
)

echo Parent finished. Check the elevated window for compaction results.
echo(
pause
exit /b 0

:CHILD
title WSL Optimize - %WSL_DISTRO%
call :IS_ADMIN
if errorlevel 1 (
  echo ERROR: Disk compaction must run from an elevated/admin command prompt.
  echo(
  pause
  exit /b 1
)

echo =========================================================
echo CHILD: Clean TEMP, shut down WSL, compact VHDX
echo =========================================================
echo(

echo [1/3] Cleaning Windows TEMP
echo TEMP=%TEMP%
if defined TEMP if exist "%TEMP%" (
  del /f /q "%TEMP%\*" >nul 2>&1
  for /d %%D in ("%TEMP%\*") do rd /s /q "%%D" >nul 2>&1
)
echo OK. Locked items may remain.
echo(

echo [2/3] Shutting down WSL
wsl.exe --shutdown >nul 2>&1
if errorlevel 1 (
  echo WARNING: wsl.exe --shutdown reported an error. Continuing.
)
echo OK.
echo(

echo [3/3] Compacting VHDX
if not exist "%VHDX%" (
  echo ERROR: VHDX not found:
  echo "%VHDX%"
  echo(
  pause
  exit /b 1
)

call :GET_VHD_GB VHD_GB_BEFORE
call :GET_FREE_GB FREE_GB_BEFORE
for %%I in ("%VHDX%") do set "VHD_BYTES_BEFORE=%%~zI"

echo Before VHDX size : %VHD_GB_BEFORE% GB
echo Before free space: %FREE_GB_BEFORE% GB
echo(

set "DPSCRIPT=%TEMP%\wsl_compact_%RANDOM%_%RANDOM%.txt"
> "%DPSCRIPT%" (
  echo select vdisk file="%VHDX%"
  echo attach vdisk readonly
  echo compact vdisk
  echo detach vdisk
  echo exit
)

echo Running DiskPart...
diskpart /s "%DPSCRIPT%"
set "ERR=%ERRORLEVEL%"
del /f /q "%DPSCRIPT%" >nul 2>&1

call :GET_VHD_GB VHD_GB_AFTER
call :GET_FREE_GB FREE_GB_AFTER
for %%I in ("%VHDX%") do set "VHD_BYTES_AFTER=%%~zI"

echo(
echo After VHDX size  : %VHD_GB_AFTER% GB
echo After free space : %FREE_GB_AFTER% GB

if not "%ERR%"=="0" (
  echo(
  echo ERROR: DiskPart failed with exit code %ERR%.
  echo(
  pause
  exit /b %ERR%
)

if "%VHD_BYTES_BEFORE%"=="%VHD_BYTES_AFTER%" (
  echo(
  echo NOTE: DiskPart completed, but the VHDX file size did not shrink.
  echo This can happen with some WSL layouts. Do not force --allow-unsafe sparse mode unless you have a backup.
) else (
  echo(
  echo Compaction changed the VHDX file size.
)

echo(
pause
endlocal
exit /b 0

:DETECT_DEFAULT_DISTRO
for /f "delims=" %%D in ('wsl.exe printenv WSL_DISTRO_NAME 2^>nul') do (
  if not defined WSL_DISTRO set "WSL_DISTRO=%%D"
)
exit /b 0

:FIND_VHDX
set "VHDX="
set "LXSS_KEY=HKCU\Software\Microsoft\Windows\CurrentVersion\Lxss"

for /f "delims=" %%K in ('reg query "%LXSS_KEY%" 2^>nul ^| findstr /r /c:"\\{.*}"') do (
  for /f "tokens=2,*" %%A in ('reg query "%%K" /v DistributionName 2^>nul ^| findstr /i /c:"DistributionName"') do (
    if /i "%%B"=="!WSL_DISTRO!" (
      for %%G in ("%%K") do set "LXSS_GUID=%%~nxG"
      if exist "%LOCALAPPDATA%\wsl\!LXSS_GUID!\ext4.vhdx" (
        set "VHDX=%LOCALAPPDATA%\wsl\!LXSS_GUID!\ext4.vhdx"
        goto :FIND_VHDX_DONE
      )
      for /f "tokens=2,*" %%C in ('reg query "%%K" /v BasePath 2^>nul ^| findstr /i /c:"BasePath"') do (
        if exist "%%D\ext4.vhdx" (
          set "VHDX=%%D\ext4.vhdx"
          goto :FIND_VHDX_DONE
        )
        if exist "%%D\LocalState\ext4.vhdx" (
          set "VHDX=%%D\LocalState\ext4.vhdx"
          goto :FIND_VHDX_DONE
        )
      )
    )
  )
)

set "FOUND_COUNT=0"
if exist "%LOCALAPPDATA%\wsl" (
  for /r "%LOCALAPPDATA%\wsl" %%F in (ext4.vhdx) do (
    set /a FOUND_COUNT+=1 >nul
    set "VHDX=%%F"
  )
)
if "!FOUND_COUNT!"=="1" goto :FIND_VHDX_DONE
if not "!FOUND_COUNT!"=="0" set "VHDX="

:FIND_VHDX_DONE
exit /b 0

:CLEAN_WSL_LOGS
set "CLEAN_SCRIPT=%TEMP%\wsl_ap_log_cleanup_%RANDOM%_%RANDOM%.sh"
set "WSL_CLEAN_SCRIPT="
> "%CLEAN_SCRIPT%" (
  echo #!/usr/bin/env bash
  echo set -euo pipefail
  echo user=$^(id -un^)
  echo home=$^(getent passwd "$user" ^| cut -d: -f6^)
  echo base="$home/ardupilot"
  echo if [[ -d "$base" ]]; then
  echo   :
  echo else
  echo   echo "No ardupilot checkout found at $base"
  echo   exit 0
  echo fi
  echo count=0
  echo while IFS= read -r -d '' dir; do
  echo   echo "Deleting $dir"
  echo   rm -rf -- "$dir"
  echo   mkdir -p -- "$dir"
  echo   count=$^(^(count + 1^)^)
  echo done ^< ^<^(find "$base" -mindepth 1 -maxdepth 2 \^( -name .git -o -name .repo \^) -prune -o -type d -name logs -print0^)
  echo echo "Cleaned $count log folders."
)

for /f "delims=" %%P in ('wsl.exe -d !WSL_DISTRO! -- wslpath -a "%CLEAN_SCRIPT%" 2^>nul') do (
  if not defined WSL_CLEAN_SCRIPT set "WSL_CLEAN_SCRIPT=%%P"
)

if not defined WSL_CLEAN_SCRIPT (
  del /f /q "%CLEAN_SCRIPT%" >nul 2>&1
  echo ERROR: Could not convert cleanup script path for WSL.
  exit /b 1
)

wsl.exe -d !WSL_DISTRO! -- sed -i "s/\r$//" "!WSL_CLEAN_SCRIPT!"
if errorlevel 1 (
  del /f /q "%CLEAN_SCRIPT%" >nul 2>&1
  echo ERROR: Could not normalize cleanup script line endings.
  exit /b 1
)

wsl.exe -d !WSL_DISTRO! -- bash "!WSL_CLEAN_SCRIPT!"
set "ERR=%ERRORLEVEL%"
del /f /q "%CLEAN_SCRIPT%" >nul 2>&1
exit /b %ERR%

:RUN_FSTRIM
wsl.exe -d !WSL_DISTRO! -u root -- sh -lc "sync; fstrim -av"
exit /b %ERRORLEVEL%

:IS_ADMIN
fltmc >nul 2>&1
exit /b %ERRORLEVEL%

:GET_VHD_GB
set "%~1="
for /f "delims=" %%G in ('powershell -NoProfile -ExecutionPolicy Bypass -Command "if(Test-Path -LiteralPath $env:VHDX){[math]::Round((Get-Item -LiteralPath $env:VHDX).Length / 1GB, 2)}"') do (
  set "%~1=%%G"
)
exit /b 0

:GET_FREE_GB
set "%~1="
for /f "delims=" %%G in ('powershell -NoProfile -ExecutionPolicy Bypass -Command "$drive=[IO.Path]::GetPathRoot($env:VHDX).Substring(0,1); [math]::Round((Get-PSDrive -Name $drive).Free / 1GB, 2)"') do (
  set "%~1=%%G"
)
exit /b 0
