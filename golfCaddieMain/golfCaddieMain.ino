/*  
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
 *   AO ----------- WRx
 *   A1 ----------- WRy
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

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
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


int leftTachPin = 2; 
unsigned long leftTachCounter = 0;

int rightTachPin = 3; 
unsigned long rightTachCounter = 0;

int time_thresh = 1;    //Set number of hall trips for RPM reading
float rpm_val = 0;
unsigned long startTime = 0;


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

// joystick = 0, RC = 1, auto = 2
int mode = 0;

void lMotor_cb( const std_msgs::Int16& cmd_msg){
  leftMotorSpeed = cmd_msg.data;
}

void rMotor_cb( const std_msgs::Int16& cmd_msg){
  rightMotorSpeed = cmd_msg.data;
}

void mode_cb( const std_msgs::Int16& cmd_msg){
  mode = cmd_msg.data;
}

//ROS subscribers for right and left servo control
ros::Subscriber<std_msgs::Int16> subLeftMotor("leftMotor", lMotor_cb);
ros::Subscriber<std_msgs::Int16> subRightMotor("rightMotor", rMotor_cb);
ros::Subscriber<std_msgs::Int16> driveMode("driveMode", mode_cb);

//BT parameters
char rxBuffer[500];
byte j = 0;
byte k = 0;

void setup() {

  //Set hallEffect pins as input
  pinMode(leftTachPin, INPUT);
  pinMode(rightTachPin, INPUT);
  
  //Attach interrupts to hall effect pins
  attachInterrupt(digitalPinToInterrupt(leftTachPin), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightTachPin), isrRight, RISING);
  
  nh.initNode();
  //Publishers
  nh.advertise(leftTachPub);
  nh.advertise(rightTachPub);
  nh.advertise(chatter);
  //Subscribers
  nh.subscribe(subLeftMotor);
  nh.subscribe(subRightMotor);
  nh.subscribe(driveMode);

  Serial.begin(57600); //usb debugging
  Serial1.begin(9600); //the bt module
}

void loop() {
  
   // Parse input string from the jetson nano

   //Joystick mode
  if (mode == 0) {
    joystickYValue = analogRead(joystickY);
    joystickXValue = analogRead(joystickX);
    
    //remap values for the motor
    leftMotorSpeed = map(joystickYValue, 0, 1023, motorMin, motorMax);
    rightMotorSpeed = map(joystickXValue, 0, 1023, motorMin, motorMax);
  }

  //RC Mode
  if(mode == 1) {

  }
  //Autonomous mode
  if(mode == 2) {
    
  }
  
  smartDriveDuo30.control(leftMotorSpeed, rightMotorSpeed);
  
  //always take tachometer reading regardless of mode
  LMotorSpeed.data = hallPinTach(leftTachCounter);
  leftTachPub.publish( &LMotorSpeed );

  RMotorSpeed.data = hallPinTach(rightTachCounter);
  rightTachPub.publish( &RMotorSpeed );
  
  nh.spinOnce();
  delay(10);
  
  btLoop();
}

//Interrupt service routine
void isrLeft() {
  leftTachCounter++;
}
void isrRight() {
  rightTachCounter++;
}
float hallPinTach(unsigned long tachCounter) {
  unsigned long endTime = micros();
  unsigned long time_passed = ((endTime - startTime) / 1000000.0);
  if (time_passed >= time_thresh) {
      rpm_val = (leftTachCounter / time_passed) * 60.0;
      leftTachCounter = 0;
      startTime = micros();
    }
    return rpm_val;
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
      
      Serial.write(rxBuffer);
      str_msg.data = rxBuffer;
  chatter.publish( &str_msg );
      for (byte m = 0; m < 100; m++) {
      rxBuffer[m] = ' ';
    }
    
    if (k > 0) Serial.println(F(" "));
    
   }  
  }
  
  //while (Serial.available()) Serial1.write(Serial.read());
  
}
