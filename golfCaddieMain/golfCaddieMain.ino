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
 *   3------------ Out
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <Cytron_SmartDriveDuo.h>

Cytron_SmartDriveDuo smartDriveDuo30(PWM_INDEPENDENT, 4, 7, 5, 6);

ros::NodeHandle  nh; //intitialize ros node handle

//Initialize message and publisher for left tachometer
std_msgs::Float32 LMotorSpeed;
ros::Publisher leftTachPub("leftTachPub", &LMotorSpeed);

int leftTachPin = 2; 
unsigned long leftTachCounter = 0;
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

void setup() {

  //Set hallEffect pins as input
  pinMode(leftTachPin, INPUT);

  //Attach interrupts to hall effect pins
  attachInterrupt(digitalPinToInterrupt(leftTachPin), isrLeft, RISING);
  
  nh.initNode();
  //Publishers
  nh.advertise(leftTachPub);
  //Subscribers
  nh.subscribe(subLeftMotor);
  nh.subscribe(subRightMotor);
  nh.subscribe(driveMode);
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
  
  nh.spinOnce();
  delay(10);
}

//Interrupt service routine
void isrLeft() {
  leftTachCounter++;
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
