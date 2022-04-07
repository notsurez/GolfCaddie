#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
 
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh; //intitialize ros node handle

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

  nh.initNode();
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
  nh.spinOnce();
  delay(10);
}
