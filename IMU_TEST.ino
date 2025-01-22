


#include <Wire.h>
#include <Servo.h>

#define DEBUG

#define motor_number 4

//Assign which pins the diffrent motors are connected to [LF, LB, RF, RB] !!!DO NOT USE PIN 9 AND 10 (Turned of when using Servo library)!!!
#define LF_PIN 3
#define LB_PIN 5
#define RF_PIN 6
#define RB_PIN 11


//initialice motors in an array [LF, LB, RF, RB]
Servo motors[motor_number];

//Array for storing which pins the motors are connected to
byte motor_pins[] = {LF_PIN, LB_PIN, RF_PIN, RB_PIN};

//array for storing values that get sent to esc [YAW, PITCH, ROLL, THROTTLE]
int input_values[4] = {1100,1100,1100,1100};

/*MPU-6050 gives you 16 bits data so you have to create some float constants
*to store the data for accelerations and gyro*/

//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
int gyro_error=0;                         //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;       //Here we store the raw data read 
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error

//Acc Variables
int acc_error=0;                            //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;         //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;         //Here we store the raw data read 
float Acc_angle_x, Acc_angle_y;             //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error

//array for storing x angle, y angle
float total_angle[2] = {0,0};

//More variables for the code
int i;
int TRIGGERED=1;
long activate_count=0;
long des_activate_count=0;

//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=0.7;//3.55
double roll_ki=0.006;//0.003
double roll_kd=1.2;//2.05
float roll_desired_angle = 0;     //This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=0.72;//3.55
double pitch_ki=0.006;//0.003
double pitch_kd=1.22;//2.05
float pitch_desired_angle = 0;     //This is the angle in which we whant the

void setup() 
{
  DDRB |= B00100000;  //D13 as output
  PORTB &= B11011111; //D13 set to LOW

  //Attach correct pins correct object in array and write 1000us to them
  for(byte i = 0; i < 4; i++)
  {
    motors[i].attach(motor_pins[i]);
    motors[i].writeMicroseconds(1000);
  }

  Init_Serial();
  Init_Wire();

  /*Here we calculate the gyro data error before we start the loop
  * make the mean of 200 values, that should be enough*/
  if(gyro_error == 0)
  {
    Calc_Gyro_Error();
  }

  /*Here we calculate the acc data error before we start the loop
 * I make the mean of 200 values, that should be enough*/
  if(acc_error==0)
  {
    Calc_Acc_Error();
  }//end of acc error calculation  
  
}//end of setup loop

void loop() 
{
  /////////////////////////////I M U/////////////////////////////////////
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;     
  /*The tiemStep is the time that elapsed since the previous loop. 
  *This is the value that we will use in the formulas as "elapsedTime" 
  *in seconds. We work in ms so we have to divide the value by 1000 
  to obtain seconds*/
  /*Reed the values that the accelerometre gives.
  * We know that the slave adress for this IMU is 0x68 in
  * hexadecimal. For that in the RequestFrom and the 
  * begin functions we have to put this value.*/   
  
  Gyro_Read();
     
  Acc_Read(); 


  //////////////////////////////////////Total angle and filter/////////////////////////////////////
  /*---X axis angle---*/
  total_angle[0] = 0.98 *(total_angle[0] + Gyro_angle_x) + 0.02*Acc_angle_x;
  /*---Y axis angle---*/
  total_angle[1] = 0.98 *(total_angle[1] + Gyro_angle_y) + 0.02*Acc_angle_y;







  /*///////////////////////////P I D///////////////////////////////////*/
  roll_desired_angle = map(input_values[2],1000,2000,-10,10);
  pitch_desired_angle = map(input_values[1],1000,2000,-10,10);

  /*First calculate the error between the desired angle and 
  *the real measured angle*/
  roll_error = total_angle[1] - roll_desired_angle;
  pitch_error = total_angle[0] - pitch_desired_angle;    
  /*Next the proportional value of the PID is just a proportional constant
  *multiplied by the error*/
  roll_pid_p = roll_kp*roll_error;
  pitch_pid_p = pitch_kp*pitch_error;
  /*The integral part should only act if we are close to the
  desired position but we want to fine tune the error. That's
  why I've made a if operation for an error between -2 and 2 degree.
  To integrate we just sum the previous integral value with the
  error multiplied by  the integral constant. This will integrate (increase)
  the value each loop till we reach the 0 point*/
  if(-3 < roll_error <3)
    {
      roll_pid_i = roll_pid_i+(roll_ki*roll_error);  
    }
  if(-3 < pitch_error <3)
    {
      pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);  
    }
  /*The last part is the derivate. The derivate acts upon the speed of the error.
  As we know the speed is the amount of error that produced in a certain amount of
  time divided by that time. For taht we will use a variable called previous_error.
  We substract that value from the actual error and divide all by the elapsed time. 
  Finnaly we multiply the result by the derivate constant*/
  roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
  pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);
  /*The final PID values is the sum of each of this 3 parts*/
  roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
  pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
  /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
  tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
  have a value of 2000us the maximum value taht we could substract is 1000 and when
  we have a value of 1000us for the PWM signal, the maximum value that we could add is 1000
  to reach the maximum 2000us. But we don't want to act over the entire range so -+400 should be enough*/
  if(roll_PID < -400){roll_PID=-400;}
  if(roll_PID > 400) {roll_PID=400; }
  if(pitch_PID < -400){pitch_PID=-400;}
  if(pitch_PID > 400) {pitch_PID=400;}

  /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
  pwm_R_F  = 115 + input_values[2] - roll_PID - pitch_PID;
  pwm_R_B  = 115 + input_values[3] - roll_PID + pitch_PID;
  pwm_L_B  = 115 + input_values[1] + roll_PID + pitch_PID;
  pwm_L_F  = 115 + input_values[0] + roll_PID - pitch_PID;



  /*Once again we map the PWM values to be sure that we won't pass the min
  and max values. Yes, we've already maped the PID values. But for example, for 
  throttle value of 1300, if we sum the max PID value we would have 2300us and
  that will mess up the ESC.*/
  //Right front
  if(pwm_R_F < 1100)
    {
      pwm_R_F= 1100;
    }
  if(pwm_R_F > 2000)
    {
      pwm_R_F=2000;
    }

  //Left front
  if(pwm_L_F < 1100)
    {
      pwm_L_F= 1100;
    }
  if(pwm_L_F > 2000)
    {
      pwm_L_F=2000;
    }

  //Right back
  if(pwm_R_B < 1100)
    {
      pwm_R_B= 1100;
    }
  if(pwm_R_B > 2000)
    {
      pwm_R_B=2000;
    }

  //Left back
  if(pwm_L_B < 1100)
    {
      pwm_L_B= 1100;
    }
  if(pwm_L_B > 2000)
    {
      pwm_L_B=2000;
    }

  roll_previous_error = roll_error; //Remember to store the previous error.
  pitch_previous_error = pitch_error; //Remember to store the previous error.

  #ifdef DEBUG
    Serial.print("RF: ");
    Serial.print(pwm_R_F);
    Serial.print("   |   ");
    Serial.print("RB: ");
    Serial.print(pwm_R_B);
    Serial.print("   |   ");
    Serial.print("LB: ");
    Serial.print(pwm_L_B);
    Serial.print("   |   ");
    Serial.print("LF: ");
    Serial.print(pwm_L_F);

    Serial.print("   |   ");
    Serial.print("Xº: ");
    Serial.print(total_angle[0]);
    Serial.print("   |   ");
    Serial.print("Yº: ");
    Serial.print(total_angle[1]);
    Serial.println(" ");
  #endif







  /*now we can write the values PWM to the ESCs only if the motor is activated
  */

  if(TRIGGERED)
    {
      motors[0].writeMicroseconds(pwm_L_F); 
      motors[1].writeMicroseconds(pwm_L_B);
      motors[2].writeMicroseconds(pwm_R_F); 
      motors[3].writeMicroseconds(pwm_R_B);
    }
  if(!TRIGGERED)
    {
      motors[0].writeMicroseconds(1000); 
      motors[1].writeMicroseconds(1000);
      motors[2].writeMicroseconds(1000); 
      motors[3].writeMicroseconds(1000);
    }
  if(!TRIGGERED)
    {
      activate_motors();
    }
}
