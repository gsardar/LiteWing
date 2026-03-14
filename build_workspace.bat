@echo off
:: ============================================================
::  build_workspace.bat — colcon build for neurotech_drone
::  Uses .venv's colcon with the combined ROS2 + .venv env.
::  Run once after any package source change.
:: ============================================================
setlocal

set "ROOT=%~dp0"

:: Source combined environment first
call "%ROOT%activate_ros.bat"
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

cd /d "%ROOT%ros2_ws"
echo.
echo Building neurotech_drone package...
echo.
colcon build --symlink-install --packages-select neurotech_drone

if %ERRORLEVEL%==0 (
    echo.
    echo [OK] Build done!  Run start_ros_nodes.bat to launch.
) else (
    echo.
    echo [FAIL] colcon build failed — see errors above.
)
