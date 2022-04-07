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
 
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh; //intitialize ros node handle

//Initialize message and publisher for left tachometer
std_msgs::Float32 LMotorSpeed;
ros::Publisher leftTachPub("leftTachPub", &LMotorSpeed);

int leftTachPin = 2; 
unsigned long leftTachCounter = 0;
int time_thresh = 1;    //Set number of hall trips for RPM reading
float rpm_val = 0;
unsigned long startTime = 0;

//Left and right servos currently sit ins for the motors
Servo myservoL;
Servo myservoR; 

// Analog pin numbers for the X and Y
const int joystickY = A0; 
const int joystickX = A1;

//Values for the min and max of the motors
const int motorMin = 0;
const int motorMax = 180;

//Servo pin for testing purposes
const int motorPinL = 9; 
const int motorPinR = 10; 

//Unmapped values
int joystickYValue = 0;
int joystickXValue = 0;

//remapped values for to assign to the motors
int xValue = 0;
int yValue = 0;

// joystick = 0, RC = 1, auto = 2
int mode = 1;

void Lservo_cb( const std_msgs::UInt16& cmd_msg){
  yValue = cmd_msg.data;
}

void Rservo_cb( const std_msgs::UInt16& cmd_msg){
  xValue = cmd_msg.data;
}

void mode_cb( const std_msgs::UInt16& cmd_msg){
  mode = cmd_msg.data;
}

//ROS subscribers for right and left servo control
ros::Subscriber<std_msgs::UInt16> subLeftServo("leftServo", Lservo_cb);
ros::Subscriber<std_msgs::UInt16> subRightServo("rightServo", Rservo_cb);
ros::Subscriber<std_msgs::UInt16> driveMode("driveMode", mode_cb);

void setup() {
  // initialize serial communications at 9600 bps:
  myservoL.attach(motorPinL);  // attaches the servo on pin 9 to the servo object
  myservoR.attach(motorPinR);

  //Set hallEffect pins as input
  pinMode(leftTachPin, INPUT);

  //Attach interrupts to hall effect pins
  attachInterrupt(digitalPinToInterrupt(leftTachPin), isrLeft, RISING);
  
  nh.initNode();
  //Publishers
  nh.advertise(leftTachPub);
  //Subscribers
  nh.subscribe(subLeftServo);
  nh.subscribe(subRightServo);
  nh.subscribe(driveMode);
}

void loop() {
  
   // Parse input string from the jetson nano

   //Joystick mode
  if (mode == 0) {
    joystickYValue = analogRead(joystickY);
    joystickXValue = analogRead(joystickX);
    
    //remap values for the motor
    yValue = map(joystickYValue, 0, 1023, motorMin, motorMax);
    xValue = map(joystickXValue, 0, 1023, motorMin, motorMax);
  }

  //RC Mode
  if(mode == 1) {

  }
  //Autonomous mode
  if(mode == 2) {
    
  }
  
  myservoL.write(yValue);
  myservoR.write(xValue);  

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
