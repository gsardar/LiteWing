# How to Flash Custom Keyboard Firmware

You previously had pre-compiled `.bin` files on the drone (`LiteWing.bin` etc) which can't be modified. I've created a completely new, open-source firmware sketch for your drone that reads commands from your python keyboard controller and directly controls the motor PWMs!

## Step 1: Physical Setup
Look at your Drone's ESP32 board. You need to identify which GPIO pins are connected to your four ESCs (motor controllers).
I've put placeholders in the code:
- `MOTOR_FL_PIN = 12; // Front Left`
- `MOTOR_FR_PIN = 13; // Front Right`
- `MOTOR_RL_PIN = 14; // Rear Left`
- `MOTOR_RR_PIN = 15; // Rear Right`

**👉 Open `firmware/keyboard_control/keyboard_control.ino` and change these pin numbers to match your drone's wiring.**

## Step 2: Install Required Libraries
Open the Arduino IDE, go to **Sketch > Include Library > Manage Libraries...**
Search for and install:
- **ESP32Servo** (Required to control motor ESCs securely)

## Step 3: Flashing the Drone ESP32
1. Plug your Drone's ESP32 directly into your computer via USB.
2. Open `firmware/keyboard_control/keyboard_control.ino` in the Arduino IDE.
3. Select the correct **Board** (likely `ESP32 Dev Module` or `ESP32-S3`).
4. Select the **Port** for your drone.
5. Click **Upload**!

## Step 4: Flashing the Bridge (if needed)
If you're using a separate "Bridge ESP32" plugged into the computer that talks to the drone wirelessly:
1. Ensure your bridge is plugged in.
2. Flash it with `drone_usb_bridge/drone_usb_bridge.ino` as usual.

## Step 5: Start Flying
1. Ensure the Drone ESP32 is powered on (and its AP `ESP32_DRONE_SWARM` is active).
2. Plug in your USB bridge ESP32.
3. Run the python script: `python keyboard_controller.py`
4. Use `W/A/S/D` and Arrows to fly!

> **Warning**: Take the propellers off when testing it for the first time! Ensure motor directions are correct before flight!
