#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <math.h>

// ---------- WiFi AP ----------
const char* ap_ssid = "DroneFCU";
const char* ap_password = "12345678";
WebServer server(80);

// ---------- I2C ----------
#define SDA_PIN 1
#define SCL_PIN 2

// ---------- Sensors ----------
Adafruit_BMP3XX bmp;
#define BMP390_ADDR 0x77
#define MPU_ADDR 0x68

const float ACC_SENS = 16384.0; // LSB/g
const float GYRO_SENS = 131.0;  // LSB/(°/s)

// ---------- Motors ----------
#define M1 12
#define M2 13
#define M3 14
#define M4 15
Servo m1, m2, m3, m4;
const int ESC_MIN = 1000;
const int ESC_MAX = 2000;
const int ESC_HOVER = 1500;

// ---------- PID ----------
float Kp_roll = 1.5, Ki_roll = 0.01, Kd_roll = 0.05;
float Kp_pitch = 1.5, Ki_pitch = 0.01, Kd_pitch = 0.05;
float Kp_yaw = 1.0, Ki_yaw = 0.005, Kd_yaw = 0.02;

// ---------- State ----------
float roll_setpoint = 0, pitch_setpoint = 0, yaw_setpoint = 0;
float roll=0, pitch=0, yaw=0;
float roll_filtered=0, pitch_filtered=0, yaw_filtered=0;

// ---------- Filter ----------
const float alpha = 0.05; // complementary + low-pass

// ---------- PID integrals/last errors ----------
float roll_integral=0, pitch_integral=0, yaw_integral=0;
float last_roll_error=0, last_pitch_error=0, last_yaw_error=0;

// ---------- MPU offsets ----------
float ax_offset=0, ay_offset=0, az_offset=0;
float gx_offset=0, gy_offset=0, gz_offset=0;

// ---------- Function declarations ----------
bool readMPU(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz);
void initMPU();
void calibrateMPU(int samples=500);
void updatePID();
void handleRoot();
void setupWiFiAP();

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.println("USB CDC ready");

  // I2C init
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Scan I2C
  Serial.println("I2C scan...");
  byte count=0;
  for(byte addr=1; addr<127; addr++){
    Wire.beginTransmission(addr);
    if(Wire.endTransmission()==0){
      Serial.print("Found device at 0x"); Serial.println(addr, HEX);
      count++;
      delay(2);
    }
  }
  if(count==0) Serial.println("No I2C devices found.");

  // BMP init
  if(!bmp.begin_I2C(BMP390_ADDR)) Serial.println("BMP390 not detected!");
  else{
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    Serial.println("BMP390 initialized.");
  }

  // MPU init + calibration
  initMPU();
  calibrateMPU();

  // Motors setup
  m1.setPeriodHertz(50);
  m2.setPeriodHertz(50);
  m3.setPeriodHertz(50);
  m4.setPeriodHertz(50);

  m1.attach(M1, ESC_MIN, ESC_MAX);
  m2.attach(M2, ESC_MIN, ESC_MAX);
  m3.attach(M3, ESC_MIN, ESC_MAX);
  m4.attach(M4, ESC_MIN, ESC_MAX);

  m1.writeMicroseconds(ESC_HOVER);
  m2.writeMicroseconds(ESC_HOVER);
  m3.writeMicroseconds(ESC_HOVER);
  m4.writeMicroseconds(ESC_HOVER);

  // WiFi AP
  setupWiFiAP();

  // Webserver
  server.on("/", handleRoot);
  server.on("/set", [](){
    if(server.hasArg("roll")) roll_setpoint = server.arg("roll").toFloat();
    if(server.hasArg("pitch")) pitch_setpoint = server.arg("pitch").toFloat();
    if(server.hasArg("yaw")) yaw_setpoint = server.arg("yaw").toFloat();
    server.send(200,"text/plain","Setpoints updated");
  });
  server.begin();
  Serial.println("Webserver started at 192.168.4.1");
}

// ---------- Main loop ----------
unsigned long lastTime=0;
void loop() {
  server.handleClient();

  unsigned long now=millis();
  float dt = (now - lastTime)/1000.0;
  if(dt<0.01) return; // ~100Hz
  lastTime = now;

  int16_t ax,ay,az,gx,gy,gz;
  if(readMPU(ax,ay,az,gx,gy,gz)){
    // Convert to g / deg/s
    float ax_g=(ax-ax_offset)/ACC_SENS;
    float ay_g=(ay-ay_offset)/ACC_SENS;
    float az_g=(az-az_offset)/ACC_SENS;
    float gx_dps=(gx-gx_offset)/GYRO_SENS;
    float gy_dps=(gy-gy_offset)/GYRO_SENS;
    float gz_dps=(gz-gz_offset)/GYRO_SENS;

    // Complementary filter
    float roll_acc = atan2(ay_g,az_g)*57.2958;
    float pitch_acc = atan2(-ax_g,sqrt(ay_g*ay_g + az_g*az_g))*57.2958;

    roll = alpha*(roll + gx_dps*dt) + (1-alpha)*roll_acc;
    pitch = alpha*(pitch + gy_dps*dt) + (1-alpha)*pitch_acc;
    yaw += gz_dps*dt;

    // Low-pass filter
    roll_filtered = alpha*roll + (1-alpha)*roll_filtered;
    pitch_filtered = alpha*pitch + (1-alpha)*pitch_filtered;
    yaw_filtered = alpha*yaw + (1-alpha)*yaw_filtered;

    // PID + motors
    updatePID();

    // Print one line
    Serial.print("Roll: "); Serial.print(roll_filtered,2);
    Serial.print(" | Pitch: "); Serial.print(pitch_filtered,2);
    Serial.print(" | Yaw: "); Serial.println(yaw_filtered,2);
  }

  // BMP optional
  if(bmp.performReading()){
    Serial.print("Temp: "); Serial.print(bmp.temperature); Serial.print(" °C | Pressure: ");
    Serial.println(bmp.pressure/100.0);
  }
}

// ---------- PID and motor control ----------
void updatePID(){
  float roll_error = roll_setpoint - roll_filtered;
  roll_integral += roll_error;
  float roll_derivative = roll_error - last_roll_error;
  last_roll_error = roll_error;

  float pitch_error = pitch_setpoint - pitch_filtered;
  pitch_integral += pitch_error;
  float pitch_derivative = pitch_error - last_pitch_error;
  last_pitch_error = pitch_error;

  float yaw_error = yaw_setpoint - yaw_filtered;
  yaw_integral += yaw_error;
  float yaw_derivative = yaw_error - last_yaw_error;
  last_yaw_error = yaw_error;

  float roll_output = Kp_roll*roll_error + Ki_roll*roll_integral + Kd_roll*roll_derivative;
  float pitch_output = Kp_pitch*pitch_error + Ki_pitch*pitch_integral + Kd_pitch*pitch_derivative;
  float yaw_output = Kp_yaw*yaw_error + Ki_yaw*yaw_integral + Kd_yaw*yaw_derivative;

  int m1_val = constrain(ESC_HOVER + roll_output + pitch_output - yaw_output, ESC_MIN, ESC_MAX);
  int m2_val = constrain(ESC_HOVER - roll_output + pitch_output + yaw_output, ESC_MIN, ESC_MAX);
  int m3_val = constrain(ESC_HOVER - roll_output - pitch_output - yaw_output, ESC_MIN, ESC_MAX);
  int m4_val = constrain(ESC_HOVER + roll_output - pitch_output + yaw_output, ESC_MIN, ESC_MAX);

  m1.writeMicroseconds(m1_val);
  m2.writeMicroseconds(m2_val);
  m3.writeMicroseconds(m3_val);
  m4.writeMicroseconds(m4_val);
}

// ---------- MPU Functions ----------
void initMPU(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void calibrateMPU(int samples){
  long ax_sum=0, ay_sum=0, az_sum=0;
  long gx_sum=0, gy_sum=0, gz_sum=0;
  Serial.println("Calibrating MPU, keep flat...");
  for(int i=0;i<samples;i++){
    int16_t ax, ay, az, gx, gy, gz;
    if(readMPU(ax,ay,az,gx,gy,gz)){
      ax_sum += ax; ay_sum += ay; az_sum += az;
      gx_sum += gx; gy_sum += gy; gz_sum += gz;
    }
    delay(5);
  }
  ax_offset = ax_sum/(float)samples;
  ay_offset = ay_sum/(float)samples;
  az_offset = az_sum/(float)samples - ACC_SENS;
  gx_offset = gx_sum/(float)samples;
  gy_offset = gy_sum/(float)samples;
  gz_offset = gz_sum/(float)samples;
  Serial.println("Calibration complete.");
}

bool readMPU(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if(Wire.endTransmission(false)!=0) return false;
  Wire.requestFrom(MPU_ADDR,14);
  if(Wire.available()<14) return false;
  ax = (Wire.read()<<8)|Wire.read();
  ay = (Wire.read()<<8)|Wire.read();
  az = (Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();
  gx = (Wire.read()<<8)|Wire.read();
  gy = (Wire.read()<<8)|Wire.read();
  gz = (Wire.read()<<8)|Wire.read();
  return true;
}

// ---------- WiFi AP setup ----------
void setupWiFiAP(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// ---------- Webserver ----------
void handleRoot(){
  String html = "<h1>Drone Control</h1>";
  html += "<form action='/set' method='get'>";
  html += "Roll: <input type='number' name='roll' step='0.1'><br>";
  html += "Pitch: <input type='number' name='pitch' step='0.1'><br>";
  html += "Yaw: <input type='number' name='yaw' step='0.1'><br>";
  html += "<input type='submit' value='Set'>";
  html += "</form>";
  server.send(200,"text/html",html);
}
