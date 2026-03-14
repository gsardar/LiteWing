@echo off
setlocal
echo ===================================================
echo Starting Neuro-Link: Full System (Arm + 3D Viz)
echo ===================================================

cd /d "%~dp0"

echo [0/3] Cleaning up...
taskkill /fi "windowtitle eq ARM_CAM" /f >nul 2>&1
taskkill /fi "windowtitle eq Neuro-Link*" /f >nul 2>&1
taskkill /fi "windowtitle eq NEURO-LINK: 3D Drone Control*" /f >nul 2>&1
timeout /t 1 /nobreak >nul

echo [1/3] Launching Arm Camera...
start "" .\scrcpy\scrcpy-win64-v3.0.2\scrcpy.exe --video-source=camera --camera-facing=back --no-audio --window-title="ARM_CAM"
timeout /t 3 /nobreak >nul

set PYTHON=python
if exist ".venv\Scripts\python.exe" set PYTHON=.venv\Scripts\python.exe

echo [2/3] Launching 3D Drone Visualizer...
start "" %PYTHON% drone_visualizer.py

echo [3/3] Launching Bionic Arm Controller...
timeout /t 2 /nobreak >nul
%PYTHON% arm_controller.py

:cleanup
echo ===================================================
echo Cleaning up...
taskkill /fi "windowtitle eq ARM_CAM" /f >nul 2>&1
taskkill /fi "windowtitle eq NEURO-LINK: 3D Drone Control*" /f >nul 2>&1
echo Done.
pause
endlocal
