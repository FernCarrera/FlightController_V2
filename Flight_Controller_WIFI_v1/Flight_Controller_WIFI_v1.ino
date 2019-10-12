//Libraries
//#include "Wire.h"
#include "Servo.h"
#include "MPU9250.h"
#include "PID.h"
#include "kalman.h"
#include "math.h"
#include <WiFiNINA.h>
#include <SPI.h>

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

//State vectors
double state[4] = {roll_sig,pitch_sig,yaw_sig,alt_sig};
double c_state[4] = {roll_ref,pitch_ref,yaw_ref,alt_ref}; //command state

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
//***************************END IMU & FILTER*****************************************

//**************************RUNTIME VARIABLES******************************************
double prev_time,curr_time,elapsed_time,time_1;
byte user_in;

short test;
//**************************END TIME VARIABLES************************************


//************************** WIFI Variables **************************************

char ssid[] = "Carrera";             //  your network SSID (name) between the " "
char pass[] = "noisydiamond257";      // your network password between the " "
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status_1 = WL_IDLE_STATUS;      //connection status
WiFiServer server(80);            //server socket

//special characters
char quote = '"';
char slash = '/';





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
  //Note:Gyro bias is calculated and removed in the .begin function
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
  Serial.println("Ready For Testing. Press '1' To Start, Press '0' To Cancel: ");

  // check for wifi shield
  checkForShield();
  // attempt to connect to wifi network
  connectToWifi();

  //Create Server
  server.begin();
  printWiFiStatus();
  
}// END SETUP

void loop() {

      //attempt to connect to wifi network
      connectToWifi();
      WiFiClient client = server.available();
      wifiController(client);
  if(test == 1){   
      //Time Calculations
      prev_time = time_1;
      curr_time = millis();
      elapsed_time = (curr_time - prev_time) / 1000; //ms to seconds conversion
    
      //get altitude reading
      //alt_sig = getAlt();
      
      //get orientation
      getState();
    
      //yaw filter
      yaw_filter.step(0,yaw_sig);
      yaw_sig = yaw_filter.current_state();
    
      //Complimentary filter
      //pitch_sig = (1 - alpha) * (pitch_sig + gx_radian) + (alpha) * (pitch_acc);
      //roll_sig = (1 - alpha) * (roll_sig + gy_radian) + (alpha) * (roll_acc);
    
      //PID Calculations
      Roll.doMath();
      Pitch.doMath();
      Yaw.doMath();
      //Alt.doMath(); //Need to implement ultrasonic sensor
    
      //print tests
      //printData();
  
      //write to motors
      pwmSend();
      Serial.print(test);
      Serial.print(",");
      Serial.print(roll_ref);
      Serial.print(",");
      Serial.print(pitch_ref);
      Serial.print(",");
      Serial.print(yaw_ref);
      Serial.print(",");
      
      // close the connection:
      client.stop();
      //Serial.println("client disonnected");
  }
  // Stop Test - Stop Motors
  else if (test==0){
    //WRITE TO THE MOTORS
    RB.writeMicroseconds(1000);
    FR.writeMicroseconds(1000);
    BL.writeMicroseconds(1000);
    FL.writeMicroseconds(1000);

    }

    
}// END LOOP


//****************************** FUNCTIONS ***************************************
//
//*********************************************************************************
//
//*********************************************************************************

// reads ultrasonic sensor and returns altitude
int getAlt(){
    digitalWrite(TRIG, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    pinMode(ECHO, INPUT);
    duration = pulseIn(ECHO, HIGH);
    alt_sig = ((duration/2) * 0.0343);     // Divide by 29.1 or multiply by 0.0343
  
    return round(alt_sig);
  
  }

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

    Serial.print(pitch_sig);
    Serial.print(",");
    Serial.print(roll_sig);
    Serial.print(",");
    Serial.println(yaw_sig);
    //Serial.print(",");
    //Serial.println(alt_out);

    }

  //CALCULATE PWM TO SEND TO MOTORS
void pwmSend(){
  
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


  //************************* WIFI FUNCTIONS ***************************************

  void connectToWifi(){
  while ( status_1 != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status_1 = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  
  }


void wifiController(WiFiClient client){
 
  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            // the content of the HTTP response follows the header:
            client.println("<!DOCTYPE HTML>");
            client.print("<head>");
            client.print("<link rel=");
            client.print(quote);
            client.print("stylesheet");
            client.print(quote);
            client.print("href=");
            client.print(quote);
            client.print("https://ioio.mah.se/courses/IDK17/student_0007/mkrsheet.css");  //NOTE: link to your own css stylesheet here
            client.print(quote);
            client.print(slash);
            client.print (">");
            client.print("</head>");
            client.print("<body>");
            client.println("<center><br><br><div class='container'><h1>Drone Orientator<h1/></div></center>");
            client.println("<center><div class='container'><left><button class='on' type='submit' value='ON' onmousedown=location.href='/H\'>ON</button>");
            client.println("<button class='off' type='submit' value='OFF' onmousedown=location.href='/L\'>OFF</button></div><br>");
            //client.println("<form action='/action_page.php'>Pitch: <input type='number' name='fname' ><br>Roll: <input type='number' name='lname'><br><input type='submit' value='Submit'></form>");
            client.println("<button class='Roll-45' type='submit' value='BLINK' onmousedown=location.href='/X\'>Roll-45</button></div>");
            client.println("<button class='Pitch-45' type='submit' value='>' onmousedown=location.href='/Y\'>Pitch-45</button></div>");
            client.println("<button class='Yaw-45' type='submit' value='<' onmousedown=location.href='/P\'>Yaw-45</button></div>");  
            client.println("<button class='Reset' type='submit' value='RESET' onmousedown=location.href='/Q\'>RESET</button></div>");  
            client.println("</body>");
            client.println("</html>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L" or "GET /X":
        if (currentLine.endsWith("GET /H")) {
          // Test ON
          test = 1;
        }
        if (currentLine.endsWith("GET /L")) {
          // Test OFF
          test = 0;
        }
         if (currentLine.endsWith("GET /P")) {
           //Roll 45
           roll_ref = 45;
        }
         if (currentLine.endsWith("GET /Y")) {
          //Pitch 45
          pitch_ref = 45;
        }
        if (currentLine.endsWith("GET /X")) {
         //yaw 45
         yaw_ref = 45;
        }
        if (currentLine.endsWith("GET /Q")) {
         //reset all orientations to 0
         roll_ref = 0;
         pitch_ref = 0;
         yaw_ref = 0;
        }

        
      }
  
  
  
  }
  }
}



void checkForShield(){
    if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present");
      while (true);       // don't continue
    }
  
  }
void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
