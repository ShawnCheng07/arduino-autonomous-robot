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

const int LIFT_SERVO_PIN = 9;
const int CLAMP_SERVO_PIN = 10;
const int LEFT_IR_SENSOR_PIN = 18;
const int CENTER_IR_SENSOR_PIN = 16;
const int RIGHT_IR_SENSOR_PIN = 14; 

// Motor constants
const int MAX_SPEED = 255; // We specified the max speed here to contrain the motor speed between the max and min
const int MIN_SPEED = 60; // The minimum motor speed that doesn't stall the motors
const int FORWARD_SPEED = 150; // Motor speed when moving straight forward 
const int TURN_SPEED = 150; // Speed for the one wheel that spins when turning

// Servo constants
const int CLAW_UP_ANGLE = 0;
const int CLAW_DOWN_ANGLE = 35;
const int CLAW_CLOSE_ANGLE = 0;
const int CLAW_OPEN_ANGLE = 110;

// Computer vision constants 
const int FRAME_CENTER = 150; // The x-position that represents the center of the camera's view
const float KP = 1.5; // Proportional gain for turning control (used for feedback)
const int TURN_THRESHOLD = 60; // How close the object must be to the center before moving forward
const int SLOW_DISTANCE = 75; // Width threshold at which the rover begins slowing down
const int GRAB_DISTANCE_WIDTH = 109; // The width of the object in pixels inside the pixycam feed

const int PIXYCAM_TRANSITION_DELAY = 150; // The delay when the rover switches from line following to pixycam
const int CLOSE_TO_LIFT_DELAY = 200; // The delay after the claw closes to when the claw lifts up 

// Debug variables
const bool DEBUG_MODE = false; // Enables the serial monitor and debug prints

// Line following variables
int program_mode = 0; // Determines which algorithm the rover is running (0:line following, 1:object tracking, 2:off)
int planned_turn_direction = 0; // What the rover will do when it can't detect the line (-1:turn left, 0:go forward, 1:turn right)

// Object detection variables 
bool within_range = false; // Whether the object is close enough to grab based on its width in the camera view
int object_x = 0;
int object_width = 0;
bool aligned = false; 

// Structures declarations
struct IRValues // A collection of bools representing all the sensors output, where (1:line detected, 0:line not found)
{
  bool left;
  bool center;
  bool right;
};

// Function declarations
IRValues get_ir_values();
void stop_motors();
void move_rover_forward();
void full_turn_rover(int direction);
void plan_direction(IRValues& ir_values);
void find_line();
void line_following();
void detect_object(); 
void move_rover();
void pixycam();


// At the start of the program this code configures pins, attaches pins to servos, positions the claw and initializes the pixycam 
void setup() 
{
  if (DEBUG_MODE)
  {
    Serial.begin(115200); // Sets the baud rate for the serial monitor
    Serial.println("Starting Program...");
  }

  // Setting the IR sensor pin as input
  pinMode(LEFT_IR_SENSOR_PIN, INPUT);
  pinMode(CENTER_IR_SENSOR_PIN, INPUT);
  pinMode(RIGHT_IR_SENSOR_PIN, INPUT);

  // Attaching pins to servo objects
  claw_clamp_servo.attach(CLAMP_SERVO_PIN); 
  claw_lift_servo.attach(LIFT_SERVO_PIN); 

  // Setting the claw to its initial position
  claw_open_lower();

  // Initialize Pixy2 camera
  pixy.init();
}

// Indefinitely checks which mode the program is in and runs their respective algorithm (0:line following, 1:object tracking, 2:off)
void loop() 
{
  // Prints a line to separate each iteration of prints 
  debug_println("----------");

  if (program_mode == 0)
  {
    // Runs the line following algorithm
    line_following();
  }
  else if (program_mode == 1)
  {
    // Runs the pixycam tracking algorithm
    pixycam(); 
  } 
}


// Line following code

// Reads and returns the IR sensor values where (1:line detected, 0:line not found)
IRValues get_ir_values()
{
  IRValues ir_values;

  // Reads the IR sensor values
  ir_values.left = digitalRead(LEFT_IR_SENSOR_PIN);
  ir_values.center = digitalRead(CENTER_IR_SENSOR_PIN);
  ir_values.right = digitalRead(RIGHT_IR_SENSOR_PIN);

  debug_println((String) ir_values.left + " " + (String) ir_values.center + " " + (String) ir_values.right);
  return ir_values;
}

// Stops the motors
void stop_motors()
{
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);

  debug_println("Stopped motors...");
}

// Moves the rover forward 
void move_rover_forward()
{
  // Ensures the motor foward speed is within the bounds of the minimum and maximum speed
  int motor_forward_speed = constrain(FORWARD_SPEED, MIN_SPEED, MAX_SPEED);

  left_motor.setSpeed(-motor_forward_speed); // The left motor is reversed since the motor setup is mirrored
  right_motor.setSpeed(motor_forward_speed);

  debug_println("Running motors forwards...");
}

// Turns the rover, by only turning one wheel so that the rover can pivot around a point
void full_turn_rover(int direction)
{
  // Ensures the motor turn speed is within the bounds of the minimum and maximum speed
  int motor_turn_speed = constrain(TURN_SPEED, MIN_SPEED, MAX_SPEED);

  if (direction < 0) // Turn left
  {
    left_motor.setSpeed(0);
    right_motor.setSpeed(motor_turn_speed);
  }
  else if (direction > 0) // Turn right
  {
    left_motor.setSpeed(-motor_turn_speed);
    right_motor.setSpeed(0);
  }

  debug_println("Fully turning motors...");
}

// Prepares a direction for when IR sensor can't find the line
void plan_direction(IRValues& ir_values)
{
  if (ir_values.left) // Left and right come before center as the rover should priotize turning when the line is missing
  {
    planned_turn_direction = -1;
  }
  else if (ir_values.right)
  {
    planned_turn_direction = 1;
  }
  else if (ir_values.center)
  {
    planned_turn_direction = 0;
  }
}

// Runs motors based on the planned turn direction
void find_line()
{
  debug_println("Finding line...");

  if (abs(planned_turn_direction) == 1) // If planned_turn_direction is left or right
  {
    full_turn_rover(planned_turn_direction); // Turns the rover in the planned direction
  }
  else
  {
    move_rover_forward();
  }
}

// Makes the rover follow a line through IR sensor data.
void line_following() 
{
  IRValues ir_values = get_ir_values();  
  plan_direction(ir_values);

  // If the IR sensor sees the end line
  if (ir_values.center && ir_values.left && ir_values.right) 
  {
    stop_motors();
    delay(PIXYCAM_TRANSITION_DELAY);
    program_mode = 1;
  }
  // If the left IR sensor sees the line
  else if (ir_values.left) 
  {
    full_turn_rover(-1);
  }
  // If the right IR sensor sees the line
  else if (ir_values.right) 
  {
    full_turn_rover(1);
  }
  // If only the center IR sensor sees the line
  else if (ir_values.center) 
  {
    move_rover_forward();
  }
  // If none of the IR sensor sees the line 
  else 
  {
    find_line();
  }
}


// Pixycam code 

// Detects the target object using the Pixy camera's color-connected components (CCC) mode.
// Updates global variables: object_x (horizontal position) and object_width (apparent size).
void detect_object()
{
  pixy.ccc.getBlocks(); // Fetch latest detected color blocks from Pixy camera

  if (pixy.ccc.numBlocks > 0) // At least one object is visible
  {
    // Store the x-position and width of the most prominent block (index 0)    
    object_x = pixy.ccc.blocks[0].m_x;
    object_width = pixy.ccc.blocks[0].m_width;

    debug_println("Object detected:" + (String) pixy.ccc.blocks[0].m_width);
  }
  else // No objects detected in frame
  {
    // Reset position and width so the rover doesn't act on stale data
    object_x = 0;
    object_width = 0;
    delay(200); // Brief pause before re-scanning to avoid rapid repeated checks

    debug_println("No object detected.");
  }
}

// Controls rover movement to centre on and approach the detected object.
// Uses the horizontal error between the object and the frame centre to steer,
// and object width as a proxy for distance.
void move_rover()
{ 
  int error = object_x - FRAME_CENTER; // Positive = object is right of centre, negative = left
  debug_println("Error:" + (String) error);

  // If the object appears large enough, it's close enough to grab — stop and return
  if (object_width >= GRAB_DISTANCE_WIDTH) 
  {
    debug_println("Object within range...");

    within_range = true;
    stop_motors();
    return;
  }

  // Object is roughly centred — drive forward
  if (abs(error) <= TURN_THRESHOLD)
  {
    aligned = true;
    debug_println("Object centered...");
    
    if (pixy.ccc.blocks[0].m_width > SLOW_DISTANCE) {
      // Object is close — reduce speed to avoid overshooting
      debug_println("Slowing down...");
      left_motor.setSpeed(-FORWARD_SPEED);
      right_motor.setSpeed(FORWARD_SPEED);
    } 
    else 
    {
      // Object is still far — drive at higher speed
      debug_println("Moving at full speed...");
      left_motor.setSpeed(-MAX_SPEED/1.5);
      right_motor.setSpeed(MAX_SPEED/1.5);
    }
  }
  else // Object is off-centre — rotate in place to re-align
  {
    aligned = false;
    int turn = constrain(KP * error, MIN_SPEED, MAX_SPEED);
    if (error == -150)
    {
      // Object location is unknown — rock the rover to find the object
      left_motor.setSpeed(-MAX_SPEED/3);
      right_motor.setSpeed(-MAX_SPEED/3);
      delay(100);
      left_motor.setSpeed(MAX_SPEED/3);
      right_motor.setSpeed(MAX_SPEED/3);
      delay(200);
    }
    else if (error < 0)
    {
      // Object is to the left — turn left in place (both motors same direction)
      left_motor.setSpeed(turn);
      right_motor.setSpeed(turn);
    }
    else if (error > 0)
    {
      // Object is to the right — turn right in place (both motors same direction)
      left_motor.setSpeed(-turn);
      right_motor.setSpeed(-turn);
    }
  }
}

// Main Pixy camera state machine — called repeatedly in the control loop.
// If the rover is within grabbing range AND an object is visible, grab it.
// Otherwise, scan for the object and steer towards it.
void pixycam() 
{
  if (within_range and pixy.ccc.numBlocks > 0) 
  {
    claw_close_lift(); // Conditions met — proceed to grab
    program_mode = 2; // This results in nothing running in the loop, effectively shutting down the program
  }
  else
  {
    detect_object(); // Update object position/size from camera
    move_rover(); // Steer and drive toward the object
  }
}


// Claw functions

// Opens and lowers the claw
void claw_open_lower()
{
  claw_clamp_servo.write(CLAW_OPEN_ANGLE);
  claw_lift_servo.write(CLAW_DOWN_ANGLE);
}

// Closes and lifts the claw
void claw_close_lift()
{
  stop_motors();
  claw_clamp_servo.write(CLAW_CLOSE_ANGLE);
  delay(CLOSE_TO_LIFT_DELAY); // Delay is to ensure the claw is fully closed before lifting 
  claw_lift_servo.write(0);

  debug_println("Grabbing object...");
}


// Debug functions

// Prints the message only if DEBUG_MODE is set to true
void debug_println(String text)
{
  if (DEBUG_MODE)
  {
    Serial.println(text);
  }
}