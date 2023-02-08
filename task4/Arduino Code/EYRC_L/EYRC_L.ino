// Declaring libraries
#include <AccelStepper.h>
#include <TimerOne.h>

// Declaring steppers
AccelStepper front_stepper(1, 6, 8);
AccelStepper right_stepper(1, 7, 9);
AccelStepper left_stepper(1, 10, 12);


// declaring variables
volatile unsigned long timer = 0;
double x = 0, y = 0, theta = 0;

void L_shape(void)
{
  
  // Moving forward 3 sec
  if (timer <= 3) 
  {
      x = 0;
      y = 400;
      theta = 0;
      delay(2000);
  }
  // Moving backward 3 sec
  else if (timer > 3 && timer <= 6) 
  {
      x = 0;
      y = -400;
      theta = 0;
      delay(2000);
  
  }
  // Moving left 3 sec
  else if (timer > 6 && timer <= 9)
  {
      x = -400;
      y = 0;
      theta = 0;
      delay(2000);
  
  }
  // Moving right 3 sec  
  else if (timer > 9 && timer <= 12)
  {
      x = 400;
      y = 0;
      theta = 0;
      delay(2000);
  
  }
  // Stopping
  else 
  {
      x = 0;
      y = 0;
      theta = 0;   
  }

  // Updating timer
  timer += 1;

  // Applying inverse kinematics
  double v_left = (0.33 * x) + (-0.58 * y) + (0.33 * theta);
  double v_right = (0.33 * x) + (0.58 * y) + (0.33 * theta);
  double v_forward = (-0.67 * x) + (0.33 * theta);

  // Setting speed to 3 steppers  
  front_stepper.setSpeed(v_forward);
  left_stepper.setSpeed(v_left);
  right_stepper.setSpeed(v_right);
  
}

void setup() 
{

  // Initializing timer to call the loop after given interval
  Timer1.initialize(1000000);
  Timer1.attachInterrupt(L_shape);

  // Setting max speed for 3 steppers
  front_stepper.setMaxSpeed(1000.0);
  left_stepper.setMaxSpeed(1000.0);
  right_stepper.setMaxSpeed(1000.0);
  
}

void loop() 
{

  // Running 3 stepper motors
  front_stepper.runSpeed();
  left_stepper.runSpeed();
  right_stepper.runSpeed();
  
}
