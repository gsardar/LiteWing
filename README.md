# LiteWing

Autonomous drone balancing and control system using ArUco markers and PID control.

## Overview

This project implements a PC-side drone balancing system. It uses a phone's camera (via scrcpy) to detect ArUco markers and sends UDP commands to an ESP32-based drone for stable flight.

## Features

- ArUco marker detection for position and orientation.
- PID controllers for Roll, Pitch, Yaw, and Thrust.
- UDP communication with drone firmware.
- Real-time telemetry and tuning GUI.
