// Declaring libraries
#include <AccelStepper.h>
#include <TimerOne.h>

// Declaring steppers 
AccelStepper front_stepper(1, 6, 8);
AccelStepper right_stepper(1, 7, 9);
AccelStepper left_stepper(1, 10, 12);

String msg = "0";

void setup() 
{
  
  // Initializing timer to call the loop after given interval
  Timer1.initialize(1000000);
  Timer1.attachInterrupt(move_hola); 

  // Setting max speed for 3 steppers
  front_stepper.setMaxSpeed(1000.0);
  left_stepper.setMaxSpeed(1000.0);
  right_stepper.setMaxSpeed(1000.0);

  Serial.begin(115200);
}

void move_hola(void)
{

  //Check if any data is available on Serial
  if(Serial.available())                   
  {   
    
    //Read message on Serial until new char(\n) which indicates end of message. Received data is stored in msg               
    msg = Serial.readStringUntil('\n');    
    int x = msg.toInt();
 
    // Setting speed to 3 steppers  
    front_stepper.setSpeed(v_forward);
    left_stepper.setSpeed(v_left);
    right_stepper.setSpeed(v_right);
    
  }  
  
}


void loop() 
{
  
  // Running 3 stepper motors
  front_stepper.runSpeed();
  left_stepper.runSpeed();
  right_stepper.runSpeed();
  
}
