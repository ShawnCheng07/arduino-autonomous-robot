// Import libraries
#include <Arduino.h>
#include <Pixy2.h>
#include <Servo.h>
#include <CytronMotorDriver.h>

// Pixy2 and servo object initializations
Pixy2 pixy;
Servo claw_clamp_servo;
Servo claw_lift_servo;

// Pin assignments
CytronMD left_motor(PWM_DIR, 5, 4);
CytronMD right_motor(PWM_DIR, 6, 7);

const int LIFT_SERVO = 10;
const int CLAMP_SERVO = 9;

const int LEFT2_IR_SENSOR = 18;
const int LEFT1_IR_SENSOR = 17;
const int CENTER_IR_SENSOR = 16;
const int RIGHT1_IR_SENSOR = 15; 
const int RIGHT2_IR_SENSOR = 14; 

// Motor constants
const int MAX_SPEED = 255;    // Maximum motor speed
const int MIN_SPEED = 50;     // Minimum motor speed (ensures the robot moves instead of stalling)

const int FORWARD_SPEED = 255; // Speed when moving forward 
const int FULL_TURN_SPEED = 200; // Speed when turning
const int SLIGHT_TURN_SPEED = 200; // Speed for the slower wheel for slight turns
const int SLOW_FACTOR = 2;

// Servo constants
const int CLAW_UP_ANGLE = 0;
const int CLAW_DOWN_ANGLE = 30;
const int CLOSE_CLAW_ANGLE = 65;
const int OPEN_CLAW_ANGLE = 110;

// Variable initialization
const int LOOP_DELAY = 0; // How long the loop waits before looping again
int current_mode = 0; // Which stage in competition the rover is in
// What direction rover will go when it can't detect the line
int planned_turn_direction = 0; // (-2: full left, -1: slight left, 0: straight, 1: slight right, 2: full right)

// Structures declarations
struct IRValues // A collection of bools representing all the sensors output
{
  bool left2;
  bool left1;
  bool center;
  bool right1;
  bool right2;
};

// Function declarations
IRValues get_ir_values();
void stop_motors();
void move_rover_forward();
void full_turn_rover(int direction);
void slight_turn_rover(int direction);
void plan_direction(IRValues& ir_values);
void find_line();
void line_following();


void setup() 
{
  Serial.begin(9600);
  Serial.println("Starting Program...");

  // Setting pin modes
  pinMode(LEFT2_IR_SENSOR, INPUT);
  pinMode(LEFT1_IR_SENSOR, INPUT);
  pinMode(CENTER_IR_SENSOR, INPUT);
  pinMode(RIGHT1_IR_SENSOR, INPUT);
  pinMode(RIGHT2_IR_SENSOR, INPUT);

  claw_clamp_servo.attach(CLAMP_SERVO);    // Attach the clamp servo 
  claw_lift_servo.attach(LIFT_SERVO);    // Attach the lift servo motor 

  claw_clamp_servo.write(OPEN_CLAW_ANGLE);    // Opens the claw
  claw_lift_servo.write(CLAW_DOWN_ANGLE);    // Tilts to down position

  pixy.init();    // Initialize Pixy2 camera
}

void loop() 
{
  Serial.println("------------");
  delay(LOOP_DELAY);
  if (current_mode == 0)
  {
    line_following();
  }
  else if (current_mode == 1) 
  {
    // Pixy cam tracking
  } 
  else if (current_mode == 2)
  {
    // Grabbing and lifting can
  } 
}

// Gets all the IR sensor data
IRValues get_ir_values()
{
  IRValues ir_values;
  ir_values.left2 = digitalRead(LEFT2_IR_SENSOR);
  ir_values.left1 = digitalRead(LEFT1_IR_SENSOR);
  ir_values.center = digitalRead(CENTER_IR_SENSOR);
  ir_values.right1 = digitalRead(RIGHT1_IR_SENSOR);
  ir_values.right2 = digitalRead(RIGHT2_IR_SENSOR);

  Serial.print(ir_values.left2);
  Serial.print(" ");
  Serial.print(ir_values.left1);
  Serial.print(" ");
  Serial.print(ir_values.center);
  Serial.print(" ");
  Serial.print(ir_values.right1);
  Serial.print(" ");
  Serial.println(ir_values.right2);
  return ir_values;
}

// Stops the motors
void stop_motors()
{
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  Serial.println("Stopped motors...");
}

// Runs the motor 
void move_rover_forward()
{
  int motor_forward_speed = constrain(FORWARD_SPEED / SLOW_FACTOR, MIN_SPEED, MAX_SPEED);

  left_motor.setSpeed(-motor_forward_speed);
  right_motor.setSpeed(motor_forward_speed);
  Serial.println("Running motors forwards...");
}

// Turns Rover
void full_turn_rover(int direction)
{
  int motor_turn_speed = constrain(FULL_TURN_SPEED / SLOW_FACTOR, MIN_SPEED, MAX_SPEED);

  if (direction < 0)
  {
    left_motor.setSpeed(motor_turn_speed);
    right_motor.setSpeed(motor_turn_speed);
  }
  else if (direction > 0)
  {
    left_motor.setSpeed(-motor_turn_speed);
    right_motor.setSpeed(-motor_turn_speed);
  }
  Serial.println("Fully turning motors...");
}

// Moves forwards and turns rover
void slight_turn_rover(int direction)
{
  int motor_fast_turn_speed = constrain(FORWARD_SPEED / SLOW_FACTOR, MIN_SPEED, MAX_SPEED);
  int motor_slow_turn_speed = constrain(SLIGHT_TURN_SPEED / SLOW_FACTOR, MIN_SPEED, MAX_SPEED);

  if (direction < 0)
  {
    left_motor.setSpeed(-motor_slow_turn_speed);
    right_motor.setSpeed(motor_fast_turn_speed);
  }
  else if (direction > 0)
  {
    left_motor.setSpeed(motor_fast_turn_speed);
    right_motor.setSpeed(-motor_slow_turn_speed);
  }
  Serial.println("Slightly turning motors...");
}

// Prepares planned direction when IR sensor can't find the line
void plan_direction(IRValues& ir_values)
{
  if (ir_values.left2)
  {
    planned_turn_direction = -2;
  }
  else if (ir_values.right2)
  {
    planned_turn_direction = 2;
  }
  else if (ir_values.center)
  {
    planned_turn_direction = 0;
  }
  else if (ir_values.left1)
  {
    planned_turn_direction = -1;
  }
  else if (ir_values.right1)
  {
    planned_turn_direction = 1;
  }
}

// Runs motors based on planned_turn_direction
void find_line()
{
  Serial.println("Finding line...");

  switch (abs(planned_turn_direction))
  {
    case 2:
      full_turn_rover(planned_turn_direction);
      break;
    
    case 1:
      slight_turn_rover(planned_turn_direction);
      break;

    default:
      move_rover_forward();
  }
}

// Makes the rover follow a line through IR sensor data.
void line_following() 
{
  IRValues ir_values = get_ir_values();  
  plan_direction(ir_values);

  if (ir_values.center && ir_values.left1 && ir_values.right1 && ir_values.left2 && ir_values.right2)
  {
    stop_motors();
    current_mode = 1;
  }
  else if (ir_values.left2)
  {
    full_turn_rover(-1);
  }
  else if (ir_values.right2)
  {
    full_turn_rover(1);
  }
  else if (ir_values.center)
  {
    move_rover_forward();
  }
  else if (ir_values.left1)
  {
    slight_turn_rover(-1);
  }
  else if (ir_values.right1)
  {
    slight_turn_rover(1);
  }
  else
  {
    find_line();
  }
}