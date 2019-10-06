//Libraries
//#include "Wire.h"
#include "Servo.h"
#include "MPU9250.h"
#include "PID.h"
#include "kalman.h"
#include "math.h"

// Servo declarations
Servo RB;
Servo FR;
Servo BL;
Servo FL;

//Definitions

//Limit PWM sent to the motors
#define MAX_SIGNAL 500
#define MIN_SIGNAL -350
#define r2d  180/PI

float pwmFL, pwmFR, pwmBL, pwmBR;


//************************** Controllers *********************************************
//************************************************************************************

double roll_sig, roll_out,roll_acc;
double pitch_sig, pitch_out,pitch_acc;
double yaw_sig, yaw_out;
double alt_sig, alt_out;

double roll_ref = 0;
double pitch_ref = 0;
double yaw_ref = 0;
double alt_ref = 152; //~5ft in cm

//Controller Gains
double kp_roll = 1;
double ki_roll = 0;
double kd_roll = 0;

double kp_pitch = 1;
double ki_pitch = 0;
double kd_pitch = 0;

double kp_yaw = 1;
double ki_yaw = 0;
double kd_yaw = 0;

double kp_alt = 0;
double ki_alt = 0;
double kd_alt = 0;

PID Roll(kp_roll, ki_roll, kd_roll, &roll_sig, &roll_out, &roll_ref, MAX_SIGNAL,MIN_SIGNAL);
PID Pitch(kp_pitch, ki_pitch, kd_pitch, &pitch_sig, &pitch_out, &pitch_ref, MAX_SIGNAL,MIN_SIGNAL);
PID Yaw(kp_yaw, ki_yaw, kd_yaw, &yaw_sig, &yaw_out, &yaw_ref, MAX_SIGNAL,MIN_SIGNAL);
PID Alt(kp_alt, ki_alt, kd_alt, &alt_sig, &alt_out, &alt_ref, MAX_SIGNAL,MIN_SIGNAL);

//************************** END CONTROLLERS *********************************************

//*************************** SENSORS & FILTERS *********************************************

MPU9250 IMU(Wire,0x68);
int status;
double accX,accY,accZ,magX,magY,magZ,gx,gy,gz,gx_radian,gy_radian;
double pitch_att,roll_att,yaw_att;

//Bias and scale (m:magnetormeter,a:accelerometer,xyz:axes, b&s: bias and scale)
double mxb = 33.89;double mxs = 0.77;double axb = 0;double axs = 1;
double myb = -9.54;double mys = 1.48;double ayb = 0;double ays = 1;
double mzb = 29.95;double mzs = 0.97;double azb = 0;double azs = 1;


//Filter
float alpha = 0.20;
float data;
double A = 1; double C = 1; double B = 0; double Q = 0.0001; double R = 1; double x = 0.5; double P = 1;
KalmanFilter yaw_filter(A, B, C, x, P, Q, R);

//Ultrasonic Sensor
int TRIG = 5;
int ECHO = 4;
long duration;
//***************************END IMU & FILTER*****************************************

//**************************TIME VARIABLES******************************************
double prev_time,curr_time,elapsed_time,time_1;
//**************************END TIME VARIABLES************************************

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  Serial.println("IMU initialization successful");

  //Set IMU calibration values setMagCalX(bias,scale)
  IMU.setMagCalX(mxb,mxs);
  IMU.setMagCalY(myb,mys);
  IMU.setMagCalZ(mzb,mzs);

  IMU.setAccelCalX(axb,axs);
  IMU.setAccelCalY(ayb,ays);
  IMU.setAccelCalZ(azb,azs);

  //Define Ultrasonic sensor pins
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);

  //Attach and Initialize motors
  RB.attach(9);
  FR.attach(5); 
  BL.attach(10);
  FL.attach(6);

  /*ensure motors dont go into config mode*/
  RB.writeMicroseconds(1000);
  FR.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);
  FL.writeMicroseconds(1000);
  
  
  delay(7000); //give the system time to set up
  time_1 = millis();  //start clock

}// END SETUP

void loop() {

  prev_time = time_1;
  curr_time = millis();
  elapsed_time = (curr_time - prev_time) / 1000; //ms to seconds conversion

  //read ultrasonic sensor
  digitalWrite(TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  pinMode(ECHO, INPUT);
  duration = pulseIn(ECHO, HIGH);
  alt_sig = ((duration/2) * 0.0343);     // Divide by 29.1 or multiply by 0.0343
  alt_sig = (int)alt_sig;

  //read IMU
  IMU.readSensor();
  //get IMU values
  accX = IMU.getAccelX_mss();
  accY = IMU.getAccelY_mss();
  accZ = IMU.getAccelZ_mss();
  magX = IMU.getMagX_uT();
  magY = IMU.getMagY_uT();
  magZ = IMU.getMagX_uT();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();

  // integrate to convert from rad/s to radian attitude
  gx_radian = gx * elapsed_time;
  gy_radian = gy * elapsed_time; 

  // Get Euler angles from IMU
  pitch_acc = atan2(accX, sqrt(accY*accY + accZ*accZ));
  roll_acc = atan2(accY, sqrt(accX*accX + accZ*accZ));
  magX = (magX*cos(pitch_acc) + magY*sin(roll_acc)*sin(pitch_acc) + magZ*cos(roll_acc)*sin(pitch_acc));
  magY = (magY * cos(roll_acc) - magZ * sin(roll_acc));
  yaw_sig = atan2(-magY,magX);
  if(yaw<0){yaw = yaw + 360;}

  //yaw filter
  yaw_filter.step(0,yaw_sig);
  yaw_sig = yaw_filter.current_state()*r2d;

  //Complimentary filter
  //pitch_sig = (1 - alpha) * (pitch_sig + gx_radian) + (alpha) * (pitch_acc);
  //roll_sig = (1 - alpha) * (roll_sig + gy_radian) + (alpha) * (roll_acc);
  pitch_sig = pitch_acc*r2d;
  roll_sig = roll_acc*r2d;

  //PID Calculations
  Roll.doMath();
  Pitch.doMath();
  Yaw.doMath();
  //Alt.doMath(); Need to implement ultrasonic sensor

  //print tests
  Serial.print(pitch_out);
  Serial.print(",");
  Serial.print(roll_out);
  Serial.print(",");
  Serial.println(yaw_out);
  //Serial.print(",");
  //Serial.println(alt_out);

  

  //CALCULATE PWM TO SEND TO MOTORS
  
  alt_out = 0; //DELETE AFTER IMPLEMENT
  pwmFR  = alt_out + roll_out - pitch_out + yaw_out;
  pwmBR  = alt_out + roll_out + pitch_out - yaw_out;
  pwmBL  = alt_out - roll_out + pitch_out + yaw_out;
  pwmFL  = alt_out - roll_out - pitch_out - yaw_out;
  
  //WRITE TO THE MOTORS
  RB.writeMicroseconds(pwmBR);
  FR.writeMicroseconds(pwmFR);
  BL.writeMicroseconds(pwmBL);
  FL.writeMicroseconds(pwmFL);

}
