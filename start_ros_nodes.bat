@echo off
:: ============================================================
::  start_ros_nodes.bat — start both Neurotech ROS2 nodes
::  Prerequisite: build_workspace.bat must have been run first.
:: ============================================================
setlocal

set "ROOT=%~dp0"

echo.
echo  =====================================================
echo   Neurotech Bionic Drone — ROS2 Node Launcher
echo  =====================================================
echo.
echo  This will open two windows:
echo    1. arm_pose_publisher  (MediaPipe arm tracking)
echo    2. drone_bridge_node   (UDP → LiteWing ESP32)
echo.
echo  Optionally add a third window for monitoring:
echo    ros2 topic echo /arm/pose
echo.

:: Source ROS2 + workspace in each new window
set "ACTIVATE=%ROOT%activate_ros.bat"

start "arm_pose_publisher" cmd /k "call ""%ACTIVATE%"" && ros2 run neurotech_drone arm_pose_publisher"
timeout /t 2 /nobreak >nul
start "drone_bridge_node"  cmd /k "call ""%ACTIVATE%"" && ros2 run neurotech_drone drone_bridge_node"
timeout /t 2 /nobreak >nul

echo Both nodes launched. Open a third terminal and run:
echo   call activate_ros.bat
echo   ros2 topic echo /arm/pose
echo.
echo Or use the launch file for both at once:
echo   ros2 launch neurotech_drone neurotech.launch.py
echo.
