
#include <Servo.h>

String myCmd[7] = {"", "", "", "", "", "", ""};

Servo myservo;

#define servopin 9




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myservo.attach(servopin);
  myservo.writeMicroseconds(1000);
  delay(7000);
  Serial.println("Start");
  
}

void loop()
{
  // put your main code here, to run repeatedly:
  while(Serial.available())
  {
    if(Serial.available() != 0)
    {
      String cmd = Serial.readStringUntil('\r');
      switch (cmd.charAt(0))
      {
        case 'A':
          myCmd[0] = cmd.substring(1);
          Serial.print("A: ");
          Serial.println(myCmd[0]);
          break;
        case 'B':
          myCmd[1] = cmd.substring(1);
          Serial.print("B: ");
          Serial.println(myCmd[1]);
          break;
        case 'C':
          myCmd[2] = cmd.substring(1);
          Serial.print("C: ");
          Serial.println(myCmd[2]);
          break;
        case 'D':
          myCmd[3] = cmd.substring(1);
          Serial.print("D: ");
          Serial.println(myCmd[3]);
          break;
        case 'E':
          myCmd[4] = cmd.substring(1);
          Serial.print("E: ");
          Serial.println(myCmd[4]);
          break;
        case 'F':
          myCmd[5] = cmd.substring(1);
          Serial.print("F: ");
          Serial.println(myCmd[5]);
          break;
        case 'G':
          myCmd[6] = cmd.substring(1);
          Serial.print("G: ");
          Serial.println(myCmd[6]);
          break;
        default:
          break;
      }
    }
  }

  if (myCmd[5] != "") {
    int val = myCmd[5].toInt();
    myservo.writeMicroseconds(val);
  }
}
