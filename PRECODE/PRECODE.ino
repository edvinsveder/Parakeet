#include <Servo.h>


Servo servo;
int channel[4];

int esc_pins[4] = {6,9,10,11};

byte chanel[4];
int throttle = 1000;

void setup() {
  
 Serial.begin(9600);
 servo.attach(9);
}

void loop() {

  throttle = analogRead(A0);
  throttle = map(throttle, 0, 255, 1000, 2000);
  servo.writeMicroseconds(throttle); // Send signal to ESC.
}
