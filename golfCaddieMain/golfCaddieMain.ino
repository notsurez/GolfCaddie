/*  z
 *  Circuit
 *  --------------------------
 *  Motor Controller:
 *   GND ---------- GND
 *   4 ------------ IN1
 *   5 ------------ AN1
 *   6 ------------ AN2
 *   7 ------------ IN2
 *   
 *   Joystick
 *   GND ---------- GND
 *   5V ----------- 5V
 *   AO ----------- VRx
 *   A1 ----------- VRy
 *   
 *   Left Tachometer
 *   GND ---------- GND
 *   5V ----------- 5V
 *   2 ------------ Out
 *   
 *   Right Tachometer
 *   GND ---------- GND
 *   5V ----------- 5V
 *   3 ------------ Out
 *   
 *   BT Module
 *   GND ---------- GND
 *   3V ----------- 3V
 *   19 ----------- TX (receive from phone)
 *   18 ----------- RX (transmit to phone)
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#define LOOPTIME  10 // looptime in ms

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Cytron_SmartDriveDuo.h>

Cytron_SmartDriveDuo smartDriveDuo30(PWM_INDEPENDENT, 4, 7, 5, 6);

ros::NodeHandle  nh; //intitialize ros node handle

//Initialize message and publisher for tachometers
std_msgs::Float32 LMotorSpeed;
ros::Publisher leftTachPub("leftTachPub", &LMotorSpeed);

std_msgs::Float32 RMotorSpeed;
ros::Publisher rightTachPub("RightTachPub", &RMotorSpeed);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);


int leftTachPin = 20; 
int rightTachPin = 21; 

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

int time_thresh = 1;    //Set number of hall trips for RPM reading
float rpm_val = 0;
unsigned long startTime = 0;


unsigned long currentMillis;
unsigned long previousMillis;

unsigned long currentMillis1;
unsigned long previousMillis1;

//Demand for linear and angular velocity
float demandx = 0;
float demandz = 0;

//Demand values for left and right motors
float demandLeft;
float demandRight;

float px, pz, cx, cz, x, z, cmd_x, cmd_z;

//
float leftEncoderDiff, rightEncoderDiff;
float leftEncoderErr, rightEncoderErr;
float leftEncoderPrev, rightEncoderPrev;

//prototype for lpf filter function
double lpf(int[], int, int);

int leftEncoderDiffLPF[16];
int leftEncoderErrLPF[80];

int rightEncoderDiffLPF[16];
int rightEncoderErrLPF[80];

// Analog pin numbers for the X and Y
const int joystickY = A0; 
const int joystickX = A1;

//Values for the min and max of the motors
const int motorMin = -100;
const int motorMax = 100;

//Servo pin for testing purposes
const int motorPinL = 9; 
const int motorPinR = 10; 

//Unmapped values
int joystickYValue = 0;
int joystickXValue = 0;

//remapped values for to assign to the motors
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

double speed_act_left = 0;
double speed_act_right = 0;

// joystick = 0, RC = 1, auto = 2
int mode = 1;

void mode_cb( const std_msgs::Int16& cmd_msg){
  mode = cmd_msg.data;
}
void velCallback(const geometry_msgs::Twist& vel) {
  cmd_x = vel.linear.x;
  cmd_z = vel.angular.z;
}
void joystickCallback(const geometry_msgs::Twist& vel) {
  px = vel.linear.x;
  pz = vel.angular.z;
}

geometry_msgs::Vector3Stamped speed_msg;  
ros::Publisher speed_pub("speed", &speed_msg);

//ROS subscribers for right and left servo control
ros::Subscriber<std_msgs::Int16> driveMode("driveMode", mode_cb);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", velCallback);
ros::Subscriber<geometry_msgs::Twist> cmd_joystick_sub("cmd_joystick", joystickCallback);

//BT parameters
char rxBuffer[500];
byte j = 0;
byte k = 0;


void setup() {

  smartDriveDuo30.control(0, 0);
  //Set hallEffect pins as input
  pinMode(leftTachPin, INPUT);
  pinMode(rightTachPin, INPUT);
  
  nh.initNode();
  //Publishers
  nh.advertise(chatter);
  nh.advertise(speed_pub);
  //Subscribers
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(cmd_joystick_sub);
  nh.subscribe(driveMode);

  Serial.begin(57600); //usb debugging
  Serial1.begin(9600); //the bt module
}

void loop() {
  nh.spinOnce();
  
   //Joystick mode
  if (mode == 1) {
    joystickYValue = analogRead(joystickY);
    joystickXValue = analogRead(joystickX);

    if(joystickYValue < 542 && joystickYValue > 482) {
      joystickYValue = 512;
    }
    if(joystickXValue < 542 && joystickXValue > 482) {
      joystickXValue = 512;
    }

    cx = 0.01*map(joystickYValue, 0, 1023, -250, 250);
    cz = 0.01*map(joystickXValue, 0, 1023, -200, 200);

    if (cx!=0){
      demandx = cx;
      demandz = cz;
    }else{
      if (cz!=0){
      demandx = cx;
      demandz = cz;
      }else if ((px == 0) && (pz == 0)){
        demandx = 0;
        demandz = 0;
      }else{
        demandx = px;
        demandz = pz;
      }
    }
  }
    
  //Autonomous 
  if(mode == 0) {
    demandx = cmd_x;
    demandz = cmd_z;
  }
  
  currentMillis = millis();
  currentMillis1 = micros();

  if(currentMillis - previousMillis >= LOOPTIME) { //1ms timed loop
    previousMillis = currentMillis;

//    if (Serial.available() > 0) {
//      char c = Serial.read();
//
//      if(c == 'w') {
//        demandx = 2;
//        demandz = 0;
//      }else if(c == 's') {
//        demandx = -2;
//        demandz = 0;
//      }else if(c =='a') {
//        //turn
//        demandx = 0;
//        demandz = 1;
//      }else if(c == 'd') {
//        demandx = 0;
//        demandz = -1;
//      }else if(c == 'z') {
//        //stop
//        demandx = 0;
//        demandz = 0;
//      }
//      
//    }
    
    //left = 1915.3
    //right 2095.4
    
    //64cm (0.64m base)
    /*
     * distance between base is 640mm, half of that is 320mm
     * Circumference is 2010.6193mm. to turnb 180 * (pi radians), each wheel drives half way
     * half = 1005.3096. half/pi = 320mm
     */
    demandLeft = demandx + (demandz*0.530);
    demandRight = demandx - (demandz*0.530);

    leftEncoderDiff = leftEncoderCount - leftEncoderPrev;
    rightEncoderDiff = rightEncoderCount - rightEncoderPrev;
    
    rightEncoderErr = (demandRight*20.95) - rightEncoderDiff;
  
    leftEncoderPrev = leftEncoderCount;
    rightEncoderPrev = rightEncoderCount;
    
    double leftDiffLPF = lpf(leftEncoderDiffLPF, 16, leftEncoderDiff);
    
    leftEncoderErr = (demandLeft*19.15) - 0.001*leftDiffLPF;
    double leftErrLPF = lpf(leftEncoderErrLPF, 80, leftEncoderErr);
    leftMotorSpeed = (int)leftErrLPF;

    double rightDiffLPF = lpf(rightEncoderDiffLPF, 16, rightEncoderDiff);
    
    rightEncoderErr = (demandRight*20.95) - 0.001*rightDiffLPF;
    double rightErrLPF = lpf(rightEncoderErrLPF, 80, rightEncoderErr);
    rightMotorSpeed = (int)rightErrLPF;

    speed_act_left = leftDiffLPF/19.15;
    speed_act_right= rightDiffLPF/20.95;
    
    Serial.print("encoderCounts:");
    Serial.print(leftEncoderDiff);
    Serial.print(",");
    Serial.print("encoderCountsLPF:");
    Serial.println(leftDiffLPF);
//    Serial.print(",");
//    Serial.print("bumpy:");
//    Serial.println(leftEncoderDiff/19.15);
    //publishSpeed(LOOPTIME);
  } //end of 10ms loop

  //80micro second loops to poll hall effects
  if(currentMillis1 - previousMillis1 >= 80) {
    previousMillis1 = currentMillis1;
    if(digitalRead(leftTachPin)){
        isrLeft();
      }
      if(digitalRead(rightTachPin)){
        isrRight();
      }
  }
  
  smartDriveDuo30.control(-leftMotorSpeed, -rightMotorSpeed);
  btLoop();
  nh.spinOnce();
  delay(1);
}

//-------------Publish function for odometry --------------
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();
  speed_msg.vector.x = speed_act_left;
  speed_msg.vector.y = speed_act_right;
  speed_msg.vector.z = time/1000;
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  //nh.loginfo("Publishing odometry");
}
//Interrupt service routine
void isrLeft() {
  if(leftMotorSpeed > 0) {
    leftEncoderCount++;
  }else if(leftMotorSpeed < 0) {
    leftEncoderCount--;
  }else{
    
  }
}

void isrRight() {
  if(rightMotorSpeed > 0) {
    rightEncoderCount++;
  }else if(rightMotorSpeed < 0) {
    rightEncoderCount--;
  }
}
double lpf(int sig[], int taps, int push) {
  double avg = 0;
  
  for(int j = taps-1; j > 0 ; j--) {
      sig[j] = sig[j-1];
    }
    sig[0] = push;

    int sum = 0;
    
    for(int i = 0; i<taps; i++) {
      sum += sig[i];
    }
    avg = sum/taps;
  return avg;
}


void btLoop() { // run over and over
  if (Serial1.available()) {
    //Serial.write(mySerial.read());
    rxBuffer[j] = Serial1.read();
    j++;
    digitalWrite(13, j%3);
    //Serial.write(rxBuffer[j]);
    //Serial.print(j);
    if (rxBuffer[j-1] == '<' || j > 100) {
      k = j - 1;
      j = 0;
      
      //Serial.write(rxBuffer);
      str_msg.data = rxBuffer;
      chatter.publish( &str_msg );
      for (byte m = 0; m < 100; m++) {
      rxBuffer[m] = ' ';
    }
    
    //if (k > 0) Serial.println(F(" "));
    
   }  
  }
  
  //while (Serial.available()) Serial1.write(Serial.read());
  
}
