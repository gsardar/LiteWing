/*
  Neuro-Link USB Keyboard Controller Bridge (ESP32)
  
  Flash this sketch to your ESP32 via Arduino IDE on COM5.
  It reads "X,Y,Z,PINCH\n" commands from USB Serial and
  forwards them via UDP to the Drone's Access Point.

  Board: ESP32-S3 (or ESP32 Dev Module)
  Baud : 115200
*/

#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "ESP32_DRONE_SWARM";
const char* password = "neuroplayground";
const char* droneIP = "192.168.4.1"; // Default IP of an ESP32 SoftAP
const int udpPort = 1234;

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.printf("Connecting to %s ...\n", ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n=== Neuro-Link USB Bridge READY ===");
    Serial.printf("Connected to Drone AP. Bridge IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[ERROR] Could not connect to Drone AP. Still accepting serial...");
  }
  
  Serial.println("Waiting for X,Y,Z,PINCH commands over Serial...");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    float x_disp, y_disp, z_elev;
    int pinch;

    if (sscanf(line.c_str(), "%f,%f,%f,%d",
               &x_disp, &y_disp, &z_elev, &pinch) == 4) {

      Serial.printf("CMD  X=%+.2f  Y=%+.2f  Z=%.2f  PINCH=%d\n",
                    x_disp, y_disp, z_elev, pinch);

      // Forward to drone via UDP
      if (WiFi.status() == WL_CONNECTED) {
        udp.beginPacket(droneIP, udpPort);
        udp.print(line); 
        udp.endPacket();
      }

    } else if (line.length() > 0) {
      Serial.print("UNKNOWN: ");
      Serial.println(line);
    }
  }
}
