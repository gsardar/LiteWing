/*
  Neuro-Link Drone Bridge (ESP32-S3)
  Receives 3D commands from Python Arm Controller
*/
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "ESP32_DRONE_SWARM"; // Drone AP
const char* password = "neuroplayground";

WiFiUDP udp;
const int localPort = 1234;
char packetBuffer[255]; 

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  udp.begin(localPort);
  Serial.println("Drone Command Bridge Ready.");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    // Parse "X,Y,Z,P"
    float x_disp, y_disp, z_elev;
    int pinch;
    if (sscanf(packetBuffer, "%f,%f,%f,%d", &x_disp, &y_disp, &z_elev, &pinch) == 4) {
      Serial.printf("POS: X=%.2f Y=%.2f Z=%.2f | PINCH=%d\n", x_disp, y_disp, z_elev, pinch);
      
      // --- APPLY TO MOTORS HERE ---
      // adjustPID(x_disp, y_disp, z_elev);
      // setThrottle(z_elev);
    }
  }

  // Handle USB Serial inputs from keyboard controller
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    float x_disp, y_disp, z_elev;
    int pinch;
    if (sscanf(line.c_str(), "%f,%f,%f,%d", &x_disp, &y_disp, &z_elev, &pinch) == 4) {
      Serial.printf("POS_USB: X=%.2f Y=%.2f Z=%.2f | PINCH=%d\n", x_disp, y_disp, z_elev, pinch);
      
      // --- APPLY TO MOTORS HERE ---
      // adjustPID(x_disp, y_disp, z_elev);
      // setThrottle(z_elev);
    }
  }
}
