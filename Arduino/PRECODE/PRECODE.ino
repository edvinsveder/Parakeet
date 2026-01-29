#include <ESP32Servo.h>

#define Motor_FRONT_LEFT  13
#define Motor_FRONT_RIGHT 12
#define Motor_REAR_LEFT   11
#define Motor_REAR_RIGHT  10

#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);

Servo motorFrontLeft;
Servo motorFrontRight;
Servo motorRearLeft;
Servo motorRearRight;

int throttle = 1000; // ESC throttle value (1000-2000)

void setup() {
  Serial.begin(115200);

  // Initialize motors (ESCs)
  motorFrontLeft.attach(Motor_FRONT_LEFT);
  motorFrontRight.attach(Motor_FRONT_RIGHT);
  motorRearLeft.attach(Motor_REAR_LEFT);
  motorRearRight.attach(Motor_REAR_RIGHT);

  // Initialize ESCs with minimum throttle
  setAllMotors(1000);
  delay(2000);

  // ESP32 as WiFi Access Point
  WiFi.softAP("ESP32_Drone_Control", "12345678");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Define web server routes
  server.on("/", handleRoot);
  server.on("/throttle", handleThrottle);
  server.on("/stop", handleStop);

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
}

void handleRoot() {
  String html = "<html><body>";
  html += "<h1>Drone Control</h1>";
  html += "<p>Current Throttle: " + String(throttle) + "</p>";
  html += "<input type='range' id='throttleSlider' min='1000' max='2000' value='" + String(throttle) + "' oninput='updateThrottle(this.value)'><br>";
  html += "<button onclick=\"setThrottle(1000)\">Min Throttle</button><br>";
  html += "<button onclick=\"setThrottle(1500)\">50% Throttle</button><br>";
  html += "<button onclick=\"setThrottle(2000)\">Max Throttle</button><br>";
  html += "<button onclick=\"fetch('/stop')\">Emergency Stop</button>";
  html += "<script>";
  html += "function updateThrottle(val) { fetch('/throttle?value=' + val); }";
  html += "function setThrottle(val) { document.getElementById('throttleSlider').value = val; fetch('/throttle?value=' + val); }";
  html += "</script>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleThrottle() {
  if (server.hasArg("value")) {
    int newThrottle = server.arg("value").toInt();
    if (newThrottle >= 1000 && newThrottle <= 2000) {
      throttle = newThrottle;
      setAllMotors(throttle);
      server.send(200, "text/plain", "Throttle set to " + String(throttle));
    } else {
      server.send(400, "text/plain", "Invalid throttle value");
    }
  } else {
    server.send(400, "text/plain", "No throttle value provided");
  }
}

void handleStop() {
  throttle = 1000;
  setAllMotors(1000);
  server.send(200, "text/plain", "Emergency stop - all motors stopped");
}

void setAllMotors(int value) {
  motorFrontLeft.writeMicroseconds(value);
  motorFrontRight.writeMicroseconds(value);
  motorRearLeft.writeMicroseconds(value);
  motorRearRight.writeMicroseconds(value);
}
