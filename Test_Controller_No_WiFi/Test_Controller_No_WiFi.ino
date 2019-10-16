//Libraries
#include "Wire.h"
#include "Servo.h"
#include "MPU9250.h"
#include "PID.h"
#include "kalman.h"
#include "math.h"
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "autoduino.h"
char val[3]; // null termination
String str;
int value;
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


//Controller Gains
double kp_roll = 2.68;
double ki_roll = 1.11;
double kd_roll = 1.61;

double kp_pitch = 2;
double ki_pitch = 0;
double kd_pitch = 0;

double kp_yaw = 2;
double ki_yaw = 0;
double kd_yaw = 0;

double kp_alt = 1.2;
double ki_alt = 0;
double kd_alt = 0;

//State vectors
int state[4] = {roll_sig,pitch_sig,yaw_sig,alt_sig};
int c_state[4] = {0,0,0,0}; //command state

double roll_ref = c_state[0];
double pitch_ref = c_state[1];
double yaw_ref = c_state[2];
double alt_ref = c_state[3]; //meters

int throttle;
bool vehicle_armed = false;
Autoduino Auto(state,c_state,&throttle,&vehicle_armed);

PID Roll(kp_roll, ki_roll, kd_roll, &roll_sig, &roll_out, &roll_ref, MAX_SIGNAL,MIN_SIGNAL);
PID Pitch(kp_pitch, ki_pitch, kd_pitch, &pitch_sig, &pitch_out, &pitch_ref, MAX_SIGNAL,MIN_SIGNAL);
PID Yaw(kp_yaw, ki_yaw, kd_yaw, &yaw_sig, &yaw_out, &yaw_ref, MAX_SIGNAL,MIN_SIGNAL);
PID Alt(kp_alt, ki_alt, kd_alt, &alt_sig, &alt_out, &alt_ref, MAX_SIGNAL,MIN_SIGNAL);

//AUTODUINO PID DECLARATION
//PID Roll(kp_roll, ki_roll, kd_roll, &roll_sig, &roll_out, &c_state[0], MAX_SIGNAL,MIN_SIGNAL);
//PID Pitch(kp_pitch, ki_pitch, kd_pitch, &pitch_sig, &pitch_out, &c_state[1], MAX_SIGNAL,MIN_SIGNAL);
//PID Yaw(kp_yaw, ki_yaw, kd_yaw, &yaw_sig, &yaw_out, &c_state[2], MAX_SIGNAL,MIN_SIGNAL);
//PID Alt(kp_alt, ki_alt, kd_alt, &alt_sig, &alt_out, &c_state[3], MAX_SIGNAL,MIN_SIGNAL);

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

//BMP280 Altimeter
Adafruit_BMP280 bmp;  //I2C declaration


//**************************RUNTIME VARIABLES******************************************
double prev_time,curr_time,elapsed_time;
byte user_in;

short test;

//**************************AUTONOMY CONTROL SETUP************************************
//bool vehicle_armed = false; 
//bool status;
bool mission_complete = false;



//************************** RADIO CONTROLLER ****************************************
const uint64_t pipeIn = 0xE8E8F0F0E1LL;     //same as in the transmitter
RF24 radio(9, 10);  //CSN and CE pins

// The sizeof this struct should not exceed 32 bytes
struct Received_data {
  byte ch1; //potentiometer
  byte ch2; //button
};

int ch1_value = 0;
int ch2_value = 0;
unsigned long last_time = 0;
Received_data received_data;
//************************** END VARIABLES    ****************************************



void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  /************COMMUNICATION WITH IMU********************************/ 
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
  //Note:Gyro bias is calculated and removed in the .begin function
  IMU.setMagCalX(mxb,mxs);
  IMU.setMagCalY(myb,mys);
  IMU.setMagCalZ(mzb,mzs);

  IMU.setAccelCalX(axb,axs);
  IMU.setAccelCalY(ayb,ays);
  IMU.setAccelCalZ(azb,azs);
  /************ BMP CHECK AND INITIALIZATION ********************************/ 
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  
  
  /*RC Controller set up*/
  radio.begin();
  //set_up();
  received_data.ch1 = 1500; // mid point throttle value
  received_data.ch2 = 0;
  
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  
  radio.openReadingPipe(1,pipeIn); 
  radio.startListening();
  
  //Attach and Initialize motors
  RB.attach(3);
  FR.attach(8); 
  BL.attach(6);
  FL.attach(5);

  /*ensure motors dont go into config mode*/
  RB.writeMicroseconds(1000);
  FR.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);
  FL.writeMicroseconds(1000);
  
  
  delay(7000); //give the system time to set up
  
  
  
}// END SETUP

void  receive_data(){
  
  while (radio.available()){
    
    radio.read(&received_data,sizeof(Received_data));
    last_time = millis();
    }
}




//****************************** FUNCTIONS ***************************************
//
//*********************************************************************************
//
//*********************************************************************************


//calculates orientation
void getState(){
  
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
    
   // pitch_sig,roll_sig,yaw_sig,alt_sig = getState();
    // integrate to convert from rad/s to radian attitude
    //gx_radian = gx * elapsed_time;
    //gy_radian = gy * elapsed_time; 
  
    // Get Euler angles from IMU
    pitch_acc = atan2(accX, sqrt(accY*accY + accZ*accZ));
    roll_acc = atan2(accY, sqrt(accX*accX + accZ*accZ));
    magX = (magX*cos(pitch_acc) + magY*sin(roll_acc)*sin(pitch_acc) + magZ*cos(roll_acc)*sin(pitch_acc));
    magY = (magY * cos(roll_acc) - magZ * sin(roll_acc));
    yaw_sig = atan2(-magY,magX);
    if(yaw_sig<0){yaw_sig = yaw_sig + 360;}

    pitch_sig = round(pitch_acc*r2d);
    roll_sig = round(roll_acc*r2d);
    yaw_sig = round(yaw_sig*r2d);
    
  
  }

  // Prints data
  void printData(){

    //Serial.print(pitch_sig);
    //Serial.print(",");
    //Serial.print(roll_sig);
    //Serial.print(",");
    //Serial.println(yaw_sig);
    //Serial.print(",");
    Serial.print("Alt-ref: "); Serial.print(c_state[3]); Serial.print(", "); Serial.print("Alt-sensor: "); Serial.print(alt_sig); Serial.print(", "); Serial.print("Alt-correct: "); Serial.print(alt_out);Serial.print(", "); Serial.print("Vehicle_armed: ");Serial.print(vehicle_armed);Serial.print(", "); Serial.print("Mission_complete: ");Serial.println(mission_complete);
    
    }

  //CALCULATE PWM TO SEND TO MOTORS
void pwmSend(){
  
    pwmFR  = alt_out + roll_out + pitch_out - yaw_out + 1000;
    pwmFL  = alt_out - roll_out + pitch_out + yaw_out + 1000;
    pwmBR  = alt_out + roll_out - pitch_out + yaw_out + 1000;
    pwmBL  = alt_out - roll_out - pitch_out - yaw_out + 1000;

  //motor checks
    if (pwmFR<1000){pwmFR=1000;}
    if (pwmFR>2000){pwmFR=2000;}
  //-----------------------------
    if (pwmFL<1000){pwmFR=1000;}
    if (pwmFL>2000){pwmFR=2000;}
  //-----------------------------
    if (pwmBR<1000){pwmFR=1000;}
    if (pwmBR>2000){pwmFR=2000;}
  //-----------------------------
    if (pwmBL<1000){pwmFR=1000;}
    if (pwmBL>2000){pwmFR=2000;}
  //-----------------------------
    
   // Serial.print(pwmFL);
    //Serial.print(",");
    //Serial.print(alt_sig);
   // Serial.print(",");
   // Serial.println(alt_out);

    //WRITE TO THE MOTORS
    RB.writeMicroseconds(pwmBR);
    FR.writeMicroseconds(pwmFR);
    BL.writeMicroseconds(pwmBL);
    FL.writeMicroseconds(pwmFL);
 
  
  }

void set_up(){
  //reset data values
  received_data.ch1 = 1500; // mid point throttle value
  received_data.ch2 = 0;
  
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  
  radio.openReadingPipe(1,pipeIn);
  }

//**********************LOOP
  void loop() {
  receive_data();
  printData();
  ch2_value = received_data.ch2;
  vehicle_armed = ch2_value;
  
  if (vehicle_armed && !mission_complete){
      /*Receive Radio Data*/
      receive_data();
      ch1_value = map(received_data.ch1,0,255,1000,2000); // throttle manual control
      ch1_value = received_data.ch1;
      ch2_value = received_data.ch2;
      vehicle_armed = ch2_value;
      
      //Serial.println(value);
     //get orientation & Altitude
      getState();
      alt_sig = bmp.readAltitude(1010)+1.4; // need to subtract the floor reading
      
      //yaw filter
      yaw_filter.step(0,yaw_sig);
      yaw_sig = yaw_filter.current_state();

      /*Define Mission*/
      Auto.alt(2);
      
      //PID Calculations
      Roll.doMath();
      Pitch.doMath();
      Yaw.doMath();
      Alt.doMath(); 
    
      //print tests
      printData();
  
      //write to motors
     // Serial.println("yoooo");
      pwmSend();
      if (Auto.completedPreviousCommand()){mission_complete=true;}
  }
  if (vehicle_armed && mission_complete){Auto.land();}
      
  else if (vehicle_armed==false){ //emergeny stop
    //WRITE TO THE MOTORS
    receive_data();
    ch1_value = received_data.ch1;
    ch2_value = received_data.ch2;
    vehicle_armed = ch2_value;
    //Auto.land();
    RB.writeMicroseconds(1000);
    FR.writeMicroseconds(1000);
    BL.writeMicroseconds(1000);
    FL.writeMicroseconds(1000);

    }

    
}// END LOOP


  
