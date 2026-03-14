@echo off
setlocal EnableDelayedExpansion
echo ===================================================
echo Starting Neurotech AI Tracker (USB Mode)
echo ===================================================

cd /d "%~dp0"

echo [0/3] Cleaning up previous processes...
taskkill /fi "windowtitle eq NEURO_CAM_SOURCE" /f >nul 2>&1
taskkill /fi "windowtitle eq Zero-Latency AI Tracker*" /f >nul 2>&1
timeout /t 1 /nobreak > nul

:: ── Check adb is available ────────────────────────────────────────────────────
where adb >nul 2>&1
if errorlevel 1 (
    if exist "%~dp0platform-tools\adb.exe" (
        set "PATH=%~dp0platform-tools;%PATH%"
        echo [OK] adb found at %~dp0platform-tools
    ) else (
        echo [ERROR] adb not found in PATH or g:\Projects\platform-tools.
        pause
        exit /b 1
    )
)

:: ── Show connected devices ────────────────────────────────────────────────────
echo [1/3] Checking phone connection...
adb devices
echo.

:: ── ADB forward ──────────────────────────────────────────────────────────────
echo [2/3] Setting up ADB port forward...
adb forward tcp:9391 tcp:9391
if errorlevel 1 (
    echo [ERROR] adb forward failed.
    pause
    exit /b 1
)
echo [OK] ADB forward tcp:9391 ready.

:: ── Check relay on phone ──────────────────────────────────────────────────────
echo       Checking relay on phone...
:check_relay
powershell -Command "try{$t=New-Object Net.Sockets.TcpClient;$t.Connect('127.0.0.1',9391);$t.Close();exit 0}catch{exit 1}" >nul 2>&1
if errorlevel 1 (
    echo.
    echo  ┌─────────────────────────────────────────────────┐
    echo  │  Open Termux on your phone and run:             │
    echo  │                                                  │
    echo  │     python relay.py                             │
    echo  │                                                  │
    echo  │  Then press ENTER here to continue.             │
    echo  └─────────────────────────────────────────────────┘
    echo.
    pause
    goto check_relay
) else (
    echo [OK] Relay is running.
)

:: ── Launch ────────────────────────────────────────────────────────────────────
echo [3/3] Launching camera engine...
start "NEURO_CAM_SOURCE" .\scrcpy\scrcpy-win64-v3.0.2\scrcpy.exe --video-source=camera --camera-facing=back --camera-size=1280x720 --no-audio --window-title=NEURO_CAM_SOURCE

echo       Launching AI Vision Processor...
timeout /t 3 /nobreak > nul

if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" track_tilt.py
) else (
    python track_tilt.py
)

:cleanup
echo ===================================================
taskkill /fi "windowtitle eq NEURO_CAM_SOURCE" /f >nul 2>&1
taskkill /fi "windowtitle eq Zero-Latency AI Tracker*" /f >nul 2>&1
echo Drone Tracker Closed.
pause
endlocal
