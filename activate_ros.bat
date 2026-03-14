@echo off
:: ============================================================
::  activate_ros.bat — Portable ROS2 Humble + .venv activator
::
::  Layers two environments together:
::   1. ROS2 ros2-windows  → provides rclpy, ros2 CLI
::   2. .venv              → provides mediapipe, opencv, win32
::
::  Usage: call activate_ros.bat
::  (run this before any ros2 / colcon command)
:: ============================================================
setlocal enabledelayedexpansion

set "ROOT=%~dp0"
set "ROS2_DIR=%ROOT%ros2\ros2-windows"
set "VENV_DIR=%ROOT%.venv"

:: ── 1. Validate paths ────────────────────────────────────────
if not exist "%ROS2_DIR%\setup.bat" (
    echo [ERROR] ROS2 not found at: %ROS2_DIR%
    exit /b 1
)
if not exist "%VENV_DIR%\Scripts\activate.bat" (
    echo [ERROR] .venv not found at: %VENV_DIR%
    exit /b 1
)

:: ── 2. Source ROS2 (provides rclpy, ros2 CLI) ────────────────
call "%ROS2_DIR%\setup.bat" 2>nul

:: ── 3. Inject .venv site-packages into PYTHONPATH ───────────
::      This makes mediapipe, opencv, pywin32 etc. available
::      to ROS2's Python without fully replacing its interpreter.
set "VENV_SP=%VENV_DIR%\Lib\site-packages"
if defined PYTHONPATH (
    set "PYTHONPATH=%VENV_SP%;%PYTHONPATH%"
) else (
    set "PYTHONPATH=%VENV_SP%"
)

:: ── 4. Put .venv\Scripts first so colcon is found ───────────
set "PATH=%VENV_DIR%\Scripts;%PATH%"

:: ── 5. Source workspace overlay (if built) ───────────────────
set "WS_SETUP=%ROOT%ros2_ws\install\local_setup.bat"
if exist "%WS_SETUP%" (
    call "%WS_SETUP%"
    echo [OK] Workspace overlay loaded.
)

echo.
echo  Environment ready:
echo    ROS2  : %ROS2_DIR%
echo    .venv : %VENV_DIR%
echo    WS    : %ROOT%ros2_ws
echo.
echo  Quick checks:
echo    ros2 --version
echo    python -c "import rclpy, mediapipe; print('All good')"
echo.
