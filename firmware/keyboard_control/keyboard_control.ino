/*
  Neuro-Link Drone Firmware - Keyboard Controlled
  
  This firmware is meant to be flashed to the ESP32 that is ON THE DRONE.
  It receives X (Roll), Y (Pitch), Z (Altitude/Throttle), and PINCH commands
  either via UDP from a USB Bridge, or directly via Serial.

  Hardware: ESP32
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// --- WiFi AP Settings ---
const char* ssid = "ESP32_DRONE_SWARM"; // Drone AP
const char* password = "neuroplayground";

WiFiUDP udp;
const int localPort = 1234;
char packetBuffer[255]; 

// --- Motor Pins ---
// IMPORTANT: Change these pins to match your actual drone's motor connections!
const int MOTOR_FL_PIN = 12; // Front Left
const int MOTOR_FR_PIN = 13; // Front Right
const int MOTOR_RL_PIN = 14; // Rear Left
const int MOTOR_RR_PIN = 15; // Rear Right

Servo motorFL;
Servo motorFR;
Servo motorRL;
Servo motorRR;

// --- Drone State ---
float current_x = 0.0; // Roll (-1.0 to 1.0)
float current_y = 0.0; // Pitch (-1.0 to 1.0)
float current_z = 0.0; // Altitude/Throttle (0.0 to 1.0)
int current_pinch = 0; // 0 or 1

unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 1000; // Cut motors if no command for 1 second

// --- Motor Mixing Parameters ---
// Adjust these to make the drone more/less aggressive
const int MIN_THROTTLE = 1000; // Minimum PWM in us (motors off)
const int MAX_THROTTLE = 2000; // Maximum PWM in us (full speed)
const int HOVER_THROTTLE = 1200; // Base throttle just before taking off. Adjust!
const float ROLL_PITCH_INFLUENCE = 200.0; // How much X/Y commands affect PWM

void setup() {
  Serial.begin(115200);
  
  // 1. Initialize Motors (ESC Calibration)
  Serial.println("Initializing Motors...");
  motorFL.attach(MOTOR_FL_PIN, MIN_THROTTLE, MAX_THROTTLE);
  motorFR.attach(MOTOR_FR_PIN, MIN_THROTTLE, MAX_THROTTLE);
  motorRL.attach(MOTOR_RL_PIN, MIN_THROTTLE, MAX_THROTTLE);
  motorRR.attach(MOTOR_RR_PIN, MIN_THROTTLE, MAX_THROTTLE);
  
  // Set all to 0 throttle
  cutMotors();
  delay(2000); // Wait for ESCs to arm
  
  // 2. Setup WiFi AP
  Serial.println("Setting up Access Point...");
  WiFi.softAP(ssid, password);
  udp.begin(localPort);
  
  Serial.println("=== Drone Keyboard-Control Firmware Ready ===");
  lastCommandTime = millis();
}

void parseCommand(float x, float y, float z, int pinch) {
  current_x = x;
  current_y = y;
  current_z = z;
  current_pinch = pinch;
  lastCommandTime = millis();
}

void cutMotors() {
  motorFL.writeMicroseconds(MIN_THROTTLE);
  motorFR.writeMicroseconds(MIN_THROTTLE);
  motorRL.writeMicroseconds(MIN_THROTTLE);
  motorRR.writeMicroseconds(MIN_THROTTLE);
}

void updateMotors() {
  // --- Safety Watchdog ---
  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    cutMotors();
    Serial.println("SAFETY CUTOFF: No commands received.");
    return;
  }
  
  // If altitude/throttle is 0, just cut motors (safety)
  if (current_z <= 0.0) {
     cutMotors();
     return;
  }
  
  // --- Mixer ---
  // Base throttle mapped from 0.0-1.0 to MIN_THROTTLE-MAX_THROTTLE
  // A Z value of 0.5 might be hover.
  float baseThrottle = map(current_z * 1000, 0, 1000, MIN_THROTTLE, MAX_THROTTLE);
  
  // X = Roll (-1.0 to 1.0). Positive X = roll right (left motors speed up, right slow down)
  float roll_effect = current_x * ROLL_PITCH_INFLUENCE;
  
  // Y = Pitch (-1.0 to 1.0). Positive Y = pitch forward (rear motors speed up, front slow down)
  // **Note**: The Python script treats 'W' as increasing Y. W usually means forward.
  float pitch_effect = current_y * ROLL_PITCH_INFLUENCE;
  
  // Mix it up!
  // Front Left:   Base - Roll + Pitch  (assuming X>0 goes right, Y>0 goes forward)
  // Front Right:  Base + Roll + Pitch
  // Rear Left:    Base - Roll - Pitch
  // Rear Right:   Base + Roll - Pitch
  // NOTE: You will likely need to invert/adjust these signs based on actual motor directions
  int pwmFL = baseThrottle + roll_effect - pitch_effect;
  int pwmFR = baseThrottle - roll_effect - pitch_effect;
  int pwmRL = baseThrottle + roll_effect + pitch_effect;
  int pwmRR = baseThrottle - roll_effect + pitch_effect;
  
  // Clamp values inside safe bounds
  pwmFL = constrain(pwmFL, MIN_THROTTLE, MAX_THROTTLE);
  pwmFR = constrain(pwmFR, MIN_THROTTLE, MAX_THROTTLE);
  pwmRL = constrain(pwmRL, MIN_THROTTLE, MAX_THROTTLE);
  pwmRR = constrain(pwmRR, MIN_THROTTLE, MAX_THROTTLE);
  
  // Apply to ESCs
  motorFL.writeMicroseconds(pwmFL);
  motorFR.writeMicroseconds(pwmFR);
  motorRL.writeMicroseconds(pwmRL);
  motorRR.writeMicroseconds(pwmRR);
}

void loop() {
  // 1. Check UDP for commands from the USB bridge
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    // Parse "X,Y,Z,P"
    float x_disp, y_disp, z_elev;
    int pinch;
    if (sscanf(packetBuffer, "%f,%f,%f,%d", &x_disp, &y_disp, &z_elev, &pinch) == 4) {
      parseCommand(x_disp, y_disp, z_elev, pinch);
      Serial.printf("UDP: X=%.2f Y=%.2f Z=%.2f P=%d\n", current_x, current_y, current_z, current_pinch);
    }
  }

  // 2. Check Serial for direct commands (if plugged in directly)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    float x_disp, y_disp, z_elev;
    int pinch;
    if (sscanf(line.c_str(), "%f,%f,%f,%d", &x_disp, &y_disp, &z_elev, &pinch) == 4) {
      parseCommand(x_disp, y_disp, z_elev, pinch);
      Serial.printf("SERIAL: X=%.2f Y=%.2f Z=%.2f P=%d\n", current_x, current_y, current_z, current_pinch);
    }
  }
  
  // 3. Apply calculated throttle/mixing to motors
  updateMotors();
  
  // Small delay to prevent swamping the loop
  delay(10);
}
