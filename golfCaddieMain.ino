#include <Servo.h>

Servo myservoL;  // create servo object to control a servo
Servo myservoR;  // create servo object to control a servo

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

// RC = 0, Auto = 1
int mode = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  myservoL.attach(motorPinL);  // attaches the servo on pin 9 to the servo object
  myservoR.attach(motorPinR);
  Serial.begin(9600);
}

void loop() {
  
   // Parse input string from the jetson nano

   //RC mode
  if (mode == 0) {
    joystickYValue = analogRead(joystickY);
    joystickXValue = analogRead(joystickX);
    
    //remap values for the motor
    yValue = map(joystickYValue, 0, 1023, 0, 180);
    xValue = map(joystickXValue, 0, 1023, 0, 180);
  }
    myservoL.write(yValue);
    myservoR.write(xValue);  

  //always take tachometer reading regardless of mode
  delay(10);
}
