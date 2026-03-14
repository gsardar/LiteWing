@echo off
setlocal
echo ===================================================
echo Starting Neuro-Link: Bionic Arm Tracker
echo ===================================================

cd /d "%~dp0"

echo [0/2] Cleaning up arm camera engine...
:: Only kill windows with the specific arm title
taskkill /fi "windowtitle eq ARM_CAM" /f >nul 2>&1
taskkill /fi "windowtitle eq Neuro-Link: Bionic Control Center*" /f >nul 2>&1
timeout /t 1 /nobreak > nul

echo [1/2] Launching Arm Camera Engine...
start "" .\scrcpy\scrcpy-win64-v3.0.2\scrcpy.exe --video-source=camera --camera-facing=back --no-audio --window-title="ARM_CAM"

echo [2/2] Launching Bionic Control Center...
timeout /t 3 /nobreak > nul

if exist ".venv\Scripts\python.exe" (
    echo Using Virtual Environment...
    ".venv\Scripts\python.exe" arm_controller.py
) else (
    echo [WARNING] .venv not found, trying system python...
    python arm_controller.py
)

:cleanup
echo ===================================================
echo Cleaning up arm processes...
taskkill /fi "windowtitle eq ARM_CAM" /f >nul 2>&1
taskkill /fi "windowtitle eq Neuro-Link: Bionic Control Center*" /f >nul 2>&1

echo.
echo Arm Tracker Closed.
pause
endlocal
