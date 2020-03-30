// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS CONTROLLED WITH YOUR SMARTPHONE
// JJROBOTS BROBOT KIT: (Arduino Mega + BROBOT ELECTRONIC BRAIN SHIELD + STEPPER MOTOR drivers)
// This code is prepared for new BROBOT shield  with ESP8266 Wifi module
// Author: JJROBOTS.COM
// Date: 02/09/2014
// Updated: 25/06/2017
// Modified: 10/05/2018 by Monty McGraw (Nikola-wan) for Arduino Mega
// Version: 2.82
// License: GPL v2
// Compiled and tested with Arduino 1.8.5. This modified version of code adds SoftwareServo (only Arduino standard libraries)
// Project URL: http://jjrobots.com/b-robot-evo-2-much-more-than-a-self-balancing-robot (Features,documentation,build instructions,how it works, SHOP,...)
// New updates:
//   - New default parameters specially tuned for BROBOT EVO 2 version (More agile, more stable...)
//   - New Move mode with position control (for externally programming the robot with a Blockly or pyhton programming interfaces)
//   - New telemtry packets send to TELEMETRY IP for monitoring Battery, Angle, ... (old battery packets for touch osc not working now)
//   - Default telemetry server is 192.168.4.2 (first client connected to the robot)
//  Get the free android app (jjrobots) from google play. For IOS users you need to use TouchOSC App + special template (info on jjrobots page)
//  Thanks to our users on the forum for the new ideas. Specially sasa999, KomX, ...

// The board needs at least 10-15 seconds with no motion (robot steady) at beginning to give good values... Robot move slightly when it´s ready!
// MPU6050 IMU connected via I2C bus. Angle estimation using complementary filter (fusion between gyro and accel)
// Angle calculations and control part is running at 100Hz

// The robot is OFF when the angle is high (robot is horizontal). When you start raising the robot it
// automatically switch ON and start a RAISE UP procedure.
// You could RAISE UP the robot also with the robot arm servo (Servo button on the interface)
// To switch OFF the robot you could manually put the robot down on the floor (horizontal)

// We use a standard PID controllers (Proportional, Integral derivative controller) for robot stability
// More info on the project page: How it works page at jjrobots.com
// We have a PI controller for speed control and a PD controller for stability (robot angle)
// The output of the control (motors speed) is integrated so it´s really an acceleration not an speed.

#include <Wire.h>
#include <SoftwareServo.h>
SoftwareServo myservo1,myservo2;  // create servo object to control two servos
#include "AS5047.h"
#include "FlySkyIBus.h"
#include <Telemetry.h>

// ---------- CALIBRATION ----------

#define ENC_1_ZERO 2485       // leg is crouched, encoder goes up from here (dirrection WILL CHANGE with back encoder)
#define ENC_2_ZERO 1210        // leg is crouched, encoder goes up from here

#define SERVO1_NEUTRAL 90 // Servo neutral position in degrees
#define SERVO1_MIN_PULSE  500
#define SERVO1_MAX_PULSE  2500
#define SERVO2_NEUTRAL 90
#define SERVO2_OFFSET 2
#define SERVO2_MIN_PULSE  500
#define SERVO2_MAX_PULSE  2500

// ---------- END CALIBRATION ----------

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 550
#define MAX_STEERING 140
#define MAX_TARGET_ANGLE 14

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO 780   // Max recommended value: 860
#define MAX_STEERING_PRO 260   // Max recommended value: 280
#define MAX_TARGET_ANGLE_PRO 26   // Max recommended value: 32

#define LEFT_KNEE_STEP 36   // Extruder 1, 1/2 stepping
#define LEFT_KNEE_DIR 34    // PORTC, 1
#define LEFT_KNEE_EN 30     // PORTC, 3

#define RIGHT_KNEE_STEP 26  // Extruder 0, 1/2 stepping
#define RIGHT_KNEE_DIR 28   // PORTA, 4
#define RIGHT_KNEE_EN 24    // PORTA, 6

#define KNEE_FULL_TURN 14400  // 200 * 8 * 9 (steps/360 * micro * belt)
#define KNEE_HALF_TURN 7200

#define ENC_1_SELECT 25  // was 16/17, but that is also TX2/RX2
#define ENC_2_SELECT 23
#define ENCODER_FULL_TURN pow(2, 14)

// Default control terms for EVO 2
//#define KP 0.32
//#define KD 0.050
//#define KP_THROTTLE 0.080
//#define KI_THROTTLE 0.1

// ROBOT TALL
//#define KP 0.27
//#define KD 0.138
//#define KP_THROTTLE 0.080
//#define KI_THROTTLE 0.1

// ROBOT MEDIUM (45)
#define KP 0.24
#define KD 0.114
#define KP_THROTTLE 0.080 
#define KI_THROTTLE 0.1

// ROBOT LOW
//#define KP 0.2
//#define KD 0.1
//#define KP_THROTTLE 0.080
//#define KI_THROTTLE 0.1


float Kpu_old;
float Kdu_old;
float Kpt_old;
float Kit_old;

#define KP_POSITION 0.06  
#define KD_POSITION 0.45  
//#define KI_POSITION 0.02

#define KNEE_KP 0.10  // 0.15 / 0.6 is best
#define KNEE_KD 0.5

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.1   
#define KD_RAISEUP 0.16   
#define KP_THROTTLE_RAISEUP 0   // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30   // Iterm windup constants for PI control 
#define ITERM_MAX 10000

#define ANGLE_OFFSET 0.0  // Offset angle for balance (to compensate robot own weight distribution)

// Telemetry
#define TELEMETRY_BATTERY 0

#define ZERO_SPEED 65535
#define MAX_ACCEL 14      // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)

#define MICROSTEPPING 16   // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)
#define KNEE_MICROSTEPPING 8

#define DEBUG 0   // 0 = No debug info (default) DEBUG 1 for console output

// AUX definitions
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

unsigned int testVal = 0;
uint8_t cascade_control_loop_counter = 0;
uint8_t loop_counter;       // To generate a medium loop 40Hz
uint8_t slow_loop_counter;  // slow loop 2Hz
uint8_t sendBattery_counter; // To send battery status
int16_t BatteryValue;
static char outstr[4];
float BatteryFloat;
float BatteryReal;

long timer_old;
long timer_value;
float debugVariable;
float dt;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;
float angle_adjusted_filtered = 0.0;
float angle_alpha = 0.05;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;
bool newControlParameters = false;
bool modifing_control_parameters = false;
int16_t position_error_sum_M1;
int16_t position_error_sum_M2;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
bool bot_enabled = false;
bool switch_pro = false;
int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
float angle_offset = ANGLE_OFFSET;
float servos_offset = 0;

boolean positionControlMode = false;
uint8_t mode;  // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

int16_t motor1;
int16_t motor2;

// position control
volatile int32_t steps1;
volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;
int16_t motor1_control;
int16_t motor2_control;

int16_t speed_M1, speed_M2;        // Actual speed of motors
int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors
int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;    // Estimated robot speed

// Knee
int16_t speed_k1, speed_k2;        // Actual speed of motors
int8_t  dir_k1, dir_k2;            // Actual direction of steppers motors
volatile int32_t steps_k1;
volatile int32_t steps_k2;
int knee1_control = 0, knee2_control = 0;
int target_steps_k1, target_steps_k2;
float kneeAngle1, kneeAngle2;
float dampen_k1 = 0, dampen_k2 = 0;

long loopCount = 0;
long loopCountStart = millis();

// Remote
int remote_chan1 = 1500;  // turn left/right
int remote_chan2 = 1500;  // forward/back
int remote_chan3 = 1000;  // up/down
int remote_chan4 = 1500;  // balance?
float kPInput = 1, kDInput = 1;

// OSC output variables
uint8_t OSCpage;
uint8_t OSCnewMessage;
float OSCfader[4];
float OSCxy1_x;
float OSCxy1_y;
float OSCxy2_x;
float OSCxy2_y;
uint8_t OSCpush[4];
uint8_t OSCtoggle[4];
uint8_t OSCmove_mode;
int16_t OSCmove_speed;
int16_t OSCmove_steps1;
int16_t OSCmove_steps2;

tAS5047 encoder1 = { .selectPin = ENC_1_SELECT };
tAS5047 encoder2 = { .selectPin = ENC_2_SELECT };

// INITIALIZATION
void setup()
{
  // Enable servos (fan MOSFET on RAMPS)
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);

//  pinMode(0, INPUT_PULLUP);   // RX pullup, required for BT HC-05 connection
  
  // STEPPER PINS ON JJROBOTS BROBOT BRAIN BOARD
  pinMode(38, OUTPUT); // ENABLE MOTOR1
  pinMode(A2, OUTPUT); // ENABLE MOTOR2
  pinMode(LEFT_KNEE_EN, OUTPUT);
  pinMode(RIGHT_KNEE_EN, OUTPUT);

  pinMode(A0, OUTPUT); // --- STEP MOTOR 1
  pinMode(A1, OUTPUT); // --- DIR MOTOR 1
  pinMode(A6, OUTPUT); // --- STEP MOTOR 2 
  pinMode(A7, OUTPUT); // --- DIR MOTOR 2

  pinMode(12, OUTPUT); // STEP MOTOR 2 PORTD,6  // WHAT IS THIS FOR?!!
  pinMode(5, OUTPUT); // DIR MOTOR 2  PORTC,6   // WHAT IS THIS FOR?!!

  // Disable motors
  digitalWrite(38, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(LEFT_KNEE_EN, HIGH);
  digitalWrite(RIGHT_KNEE_EN, HIGH);
  
  Serial.begin(115200); // Serial output to console

  // Initialize I2C bus (MPU6050 is connected via I2C)
  Wire.begin();

#if DEBUG > 0
  delay(2000);
#else
  delay(1000);
#endif
  Serial.println("JJROBOTS");
  delay(200);
  MPU6050_setup();  // setup MPU6050 IMU

  // Calibrate gyros, pass 0 to force calibration
  MPU6050_calibrate(-295);

  // Init servos
  Serial.println("Servo init");
//  BROBOT_initServo();

  myservo1.attach(4);
  myservo1.setMinimumPulse(SERVO1_MIN_PULSE);
  myservo1.setMaximumPulse(SERVO1_MAX_PULSE);
  myservo2.attach(5);
  myservo2.setMinimumPulse(SERVO2_MIN_PULSE);
  myservo2.setMaximumPulse(SERVO2_MAX_PULSE);

  // setup encoders
  AS5047_SPI_Init();
  AS5047_Init(encoder1);
  AS5047_Init(encoder2);

  // STEPPER MOTORS INITIALIZATION
  Serial.println("Steppers init");
  //MOTOR1 => TIMER1
  TCCR1A = 0;                       // Timer1 CTC mode 4, OCxA,B outputs disconnected
  TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler=8, => 2Mhz
  OCR1A = ZERO_SPEED;               // Motor stopped
  dir_M1 = 0;
  TCNT1 = 0;

  // MOTOR2 => TIMER3
  TCCR3A = 0;                       // Timer3 CTC mode 4, OCxA,B outputs disconnected
  TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
  OCR3A = ZERO_SPEED;   // Motor stopped
  dir_M2 = 0;
  TCNT3 = 0;

  // KNEE STEPPERS INITIALIZATION
  // MOTOR1 => TIMER4
  TCCR4A = 0;                       // Timer4 CTC mode 4, OCxA,B outputs disconnected
  TCCR4B = (1 << WGM12) | (1 << CS11); // Prescaler=8, => 2Mhz
  OCR4A = ZERO_SPEED;               // Motor stopped
  dir_k1 = 0;
  TCNT4 = 0;

  // MOTOR2 => TIMER5
  TCCR5A = 0;                       // Timer5 CTC mode 4, OCxA,B outputs disconnected
  TCCR5B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
  OCR5A = ZERO_SPEED;   // Motor stopped
  dir_k2 = 0;
  TCNT5 = 0;  
  delay(200);

  // Enable stepper drivers and TIMER interrupts
  digitalWrite(38, LOW);   // Enable stepper drivers
  digitalWrite(A2, LOW);
  digitalWrite(LEFT_KNEE_EN, LOW);
  digitalWrite(RIGHT_KNEE_EN, LOW);
  
  // Enable TIMERs interrupts
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
  TIMSK3 |= (1 << OCIE1A); // Enable Timer3 interrupt
  TIMSK4 |= (1 << OCIE4A); // Enable Timer4 interrupt
  TIMSK5 |= (1 << OCIE5A); // Enable Timer5 interrupt

#if TELEMETRY_BATTERY==1
  BatteryValue = BROBOT_readBattery(true);
  Serial.print("BATT:");
  Serial.println(BatteryValue);
#endif
  Serial.println("Start...");
  timer_old = micros();

  IBus.begin(Serial1);

  Telemetry.attach_f32_to("alpha", &angle_alpha);
  Telemetry.attach_f32_to("ao", &angle_offset);
  Telemetry.attach_f32_to("so", &servos_offset);
}


// MAIN LOOP
void loop()
{
  loopCount++;
  IBus.loop();

  Telemetry.update();
  Telemetry.pub_f32("alpha", angle_alpha);
  Telemetry.pub_f32("ao", angle_offset);
  Telemetry.pub_f32("so", servos_offset);

  readEncoders();
  syncKneeSteppers(20);  // max tolerated error
  
  if (OSCnewMessage)
  {
    OSCnewMessage = 0;
    if (OSCpage == 1)   // Get commands from user (PAGE1 are user commands: throttle, steering...)
    {
      if (modifing_control_parameters)  // We came from the settings screen
      {
        OSCfader[0] = 0.5; // default neutral values
        OSCfader[1] = 0.5;
        OSCtoggle[0] = 0;  // Normal mode
        mode = 0;
        modifing_control_parameters = false;
      }

      if (OSCmove_mode)
      {
        //Serial.print("M ");
        //Serial.print(OSCmove_speed);
        //Serial.print(" ");
        //Serial.print(OSCmove_steps1);
        //Serial.print(",");
        //Serial.println(OSCmove_steps2);
        positionControlMode = true;
        OSCmove_mode = false;
        target_steps1 = steps1 + OSCmove_steps1;
        target_steps2 = steps2 + OSCmove_steps2;
      }
      else
      {
        positionControlMode = false;
        throttle = (OSCfader[0] - 0.5) * max_throttle;
        // We add some exponential on steering to smooth the center band
        steering = OSCfader[1] - 0.5;
        if (steering > 0)
          steering = (steering * steering + 0.5 * steering) * max_steering;
        else
          steering = (-steering * steering + 0.5 * steering) * max_steering;
      }

      if ((mode == 0) && (switch_pro))
      {
        // Change to PRO mode
        max_throttle = MAX_THROTTLE_PRO;
        max_steering = MAX_STEERING_PRO;
        max_target_angle = MAX_TARGET_ANGLE_PRO;
        mode = 1;
      }
      if ((mode == 1) && (!switch_pro))
      {
        // Change to NORMAL mode
        max_throttle = MAX_THROTTLE;
        max_steering = MAX_STEERING;
        max_target_angle = MAX_TARGET_ANGLE;
        mode = 0;
      }
    }
    else if (OSCpage == 2) { // OSC page 2
      // Check for new user control parameters
//      readControlParameters();
    }
#if DEBUG==1
    Serial.print(throttle);
    Serial.print(" ");
    Serial.println(steering);
#endif
  } // End new OSC message

  timer_value = micros();

  // New IMU data?
  if (MPU6050_newData())
  {
    MPU6050_read_3axis();
    loop_counter++;
    slow_loop_counter++;
    dt = (timer_value - timer_old) * 0.000001; // dt in seconds
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    float MPU_sensor_angle = MPU6050_getAngle(dt);
    angle_adjusted = MPU_sensor_angle + angle_offset;
    if ((MPU_sensor_angle>-15)&&(MPU_sensor_angle<15))
      angle_adjusted_filtered = angle_adjusted_filtered*(1-angle_alpha) + MPU_sensor_angle*angle_alpha;
      
#if DEBUG==1
    Serial.print(dt);
    Serial.print(" ");
    Serial.print(angle_offset);
    Serial.print(" ");
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.println(angle_adjusted_filtered);
#endif
    Telemetry.pub_f32("aa", angle_adjusted);
    Telemetry.pub_f32("aaf", angle_adjusted_filtered);
    //Serial.print("\t");

    // We calculate the estimated robot speed:
    // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward  

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed + angular_velocity;
    estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // low pass filter on estimated speed

#if DEBUG==2
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.println(estimated_speed_filtered);
#endif

    if (positionControlMode)
    {
      // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
      motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
      motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

      // Convert from motor position control to throttle / steering commands
      throttle = (motor1_control + motor2_control) / 2;
      throttle = constrain(throttle, -190, 190);
      steering = motor2_control - motor1_control;
      steering = constrain(steering, -50, 50);
    }

    // ROBOT SPEED CONTROL: This is a PI controller.
    //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output


#if DEBUG==3
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.print(estimated_speed_filtered);
    Serial.print(" ");
    Serial.println(target_angle);
#endif

    // Stability control (100Hz loop): This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp * kPInput, Kd * kDInput);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly to the output
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    // Knees
    knee1_control = positionPDControl(steps_k1, target_steps_k1, KNEE_KP, KNEE_KD, speed_k1);
    setMotorSpeedK1(constrain(knee1_control, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT));
    knee2_control = positionPDControl(steps_k2, target_steps_k2, KNEE_KP, KNEE_KD, speed_k2);
    setMotorSpeedK2(constrain(knee2_control, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT));

    int angle_ready;
    if (OSCpush[0])     // If we press the SERVO button we start to move
      angle_ready = 82;
    else
      angle_ready = 74;  // Default angle
    if (bot_enabled && (angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready)) // Is robot ready (upright?)
    {
      // NORMAL MODE
      digitalWrite(38, LOW);  // Motors enable
      digitalWrite(A2, LOW);
      digitalWrite(LEFT_KNEE_EN, LOW);
      digitalWrite(RIGHT_KNEE_EN, LOW);
      // NOW we send the commands to the motors
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
    }
    else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
    {
      digitalWrite(38, HIGH);  // Disable motors
      digitalWrite(A2, HIGH);
      digitalWrite(LEFT_KNEE_EN, HIGH);
      digitalWrite(RIGHT_KNEE_EN, HIGH);
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      setMotorSpeedK1(0);
      setMotorSpeedK2(0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      // RESET steps
      steps1 = 0;
      steps2 = 0;
      positionControlMode = false;
      OSCmove_mode = false;
      throttle = 0;
      steering = 0;
    }

    if (IBus.isActive()) {
      float alpha = 0.2;
      remote_chan1 = IBus.readChannel(0) * alpha + remote_chan1 * (1-alpha);
      remote_chan2 = IBus.readChannel(1) * alpha + remote_chan2 * (1-alpha);
      remote_chan3 = IBus.readChannel(2) * alpha + remote_chan3 * (1-alpha);
      remote_chan4 = IBus.readChannel(3) * alpha + remote_chan4 * (1-alpha);

      // TODO: this should ber reversed
      steering = -1 * ((remote_chan1 - 1500) / 1000.0) * max_steering;
      throttle = -1 * ((remote_chan2 - 1500) / 1000.0) * max_throttle;

      bot_enabled = IBus.readChannel(6) > 1999;
      switch_pro = IBus.readChannel(7) > 1999;

      kPInput = ((float) IBus.readChannel(4)-1000)/500.0;  // normalize between 0 and 2 (-100% / +100%)
      kDInput = ((float) IBus.readChannel(5)-1000)/500.0;

      int microsOffset = remote_chan3 - 1000;  // DS3225 Pulse width range: 500~2500 μsec
      float balanceOffset = (remote_chan4 - 1500) / 500.0 * 0.1;
      float height = microsOffset / 1000.0f;  // should be 1 - ...
      Telemetry.pub_f32("h", height);

      target_steps_k1 = constrain((height + balanceOffset) * KNEE_HALF_TURN, 0, KNEE_HALF_TURN);
      target_steps_k2 = constrain((height - balanceOffset) * KNEE_HALF_TURN, 0, KNEE_HALF_TURN);

      myservo1.write(constrain(SERVO1_NEUTRAL + servos_offset + 90 * (1-abs(steps_k1)/(float)KNEE_HALF_TURN), 70, 180));
      myservo2.write(constrain(SERVO2_NEUTRAL - servos_offset + SERVO2_OFFSET - 90 * (1-abs(steps_k2)/(float)KNEE_HALF_TURN), 0, 110));
//      myservo1.writeMicroseconds(constrain(SERVO1_NEUTRAL + 1000 - (abs(steps_k1) / 1.8), 1300, 2500));
//      myservo2.writeMicroseconds(constrain(SERVO2_NEUTRAL - 1000 + steps_k2 / 1.8, 500, 1700));
      if (bot_enabled) {
        SoftwareServo::refresh();
      }
    }

    // Normal condition?
    if ((angle_adjusted < 56) && (angle_adjusted > -56))
    {
      Kp = Kp_user;            // Default user control gains
      Kd = Kd_user;
      Kp_thr = Kp_thr_user;
      Ki_thr = Ki_thr_user;
    }
    else    // We are in the raise up procedure => we use special control parameters
    {
      Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
    }

  } // End of new IMU data

  // Medium loop 7.5Hz
  if (loop_counter >= 15)
  {
    loop_counter = 0;

#if DEBUG==7
    // How fast is the main loop?
    Serial.print(loopCount * 1000 / (millis() - loopCountStart));
    Serial.println(" Hz");
    loopCount = 0; loopCountStart = millis();
#endif

    // Telemetry here?
#if TELEMETRY_ANGLE==1
    char auxS[25];
    int ang_out = constrain(int(angle_adjusted * 10), -900, 900);
    sprintf(auxS, "$tA,%+04d", ang_out);
    Serial1.println(auxS);
#endif
#if TELEMETRY_DEBUG==1
    char auxS[50];
    sprintf(auxS, "$tD,%d,%d,%ld", int(angle_adjusted * 10), int(estimated_speed_filtered), steps1);
    Serial1.println(auxS);
#endif

  } // End of medium loop
  else if (slow_loop_counter >= 100) // 1Hz
  {
    slow_loop_counter = 0;
    // Read  status
#if TELEMETRY_BATTERY==1
    int BatteryValue = (BatteryValue + BROBOT_readBattery(false)) / 2;
    sendBattery_counter++;
    if (sendBattery_counter >= 3) { //Every 3 seconds we send a message
      sendBattery_counter = 0;
      char OSCaddress[32] = {"/1/label5\0\0\0,f\0\0\0\0\0\0"};
      char OSCdata[5] = {" red"};
      float BatteryReal = BatteryValue / 10.0f;
      dtostrf(BatteryReal, 4, 1, OSCdata);
      //now update the battery /1/rotary1 graph by scaling batteryfloat to Max and Min battery values
#if LIPOBATT==0       //Alkaline - warn at 1.1Volts per cell = 6.6V pack  Min=1Vpc 6.0V pack, Max=1.5Vpc 9.0V pack
      // From >10.6 volts (100%) to 9.2 volts (0%) (aprox) ---- this comment and float are from original BROBOT.INO
      // float value = constrain((BatteryValue-92)/14.0,0.0,1.0);

      // From >9.0 volts (100%) to 6.0 volts (0%) (aprox)  for AA Alkaline battery pack of 6
      //float value = constrain((BatteryValue - 60) / 30.0, 0.0, 1.0);
      float value = 0.6f;
#else
      // For Lipo battery use better this config: (From >11.5v (100%) to 9.5v (0%)
      //float value = constrain((BatteryValue - 95) / 20.0, 0.0, 1.0);
      float value = 0.6f;
#endif
      //  Battery data output to phone is now in OSCxy module - only sent after OSC ping is received by phone
      // OSC_MsgSend("/1/fader8\0\0\0,f\0\0\0\0\0\0",20,value);
      Serial.print("value:");
      Serial.println(BatteryValue);
      Serial.print("BatteryReal:");
      //dtostrf(BatteryReal, 4, 1, OSCdata);
      //Serial.println(OSCdata);
      // OSC_StrSend("/1/label8\0\0\0,s\0\0red\0",20,OSCdata);
      
    }
#endif
  }  // End of slow loop
}

void readEncoders() {
  float alpha = 0.1;
  AS5047_Read(encoder1, CMD_R_ANGLECOM);
  float newAngle1 = (encoder1.data.ANGLECOM.DAECANG - ENC_1_ZERO) / ENCODER_FULL_TURN * 360;
  kneeAngle1 = alpha * newAngle1 + (1-alpha) * kneeAngle1;

  AS5047_Read(encoder2, CMD_R_ANGLECOM);
  float newAngle2 = (encoder2.data.ANGLECOM.DAECANG - ENC_2_ZERO) / ENCODER_FULL_TURN * 360;
  kneeAngle2 = alpha * newAngle2 + (1-alpha) * kneeAngle2;
/*
  Serial.print(kneeAngle1);
  Serial.print("\t");
  Serial.println(kneeAngle2);
*/
};

float avgDiff1 = 1;

void syncKneeSteppers(int tolerance) {
  int expectedSteps1 = round(kneeAngle1 / 360 * KNEE_FULL_TURN);
  int expectedSteps2 = round(kneeAngle2 / 360 * KNEE_FULL_TURN);

  int diff1 = steps_k1 - expectedSteps1;
  float alpha = 0.01;
  avgDiff1 = alpha * diff1 + (1-alpha) * avgDiff1;
/*
  Serial.print(diff1);
  Serial.print("\t");
  Serial.print(avgDiff1);
  Serial.print("\t");
//  Serial.print(steps_k2 - expectedSteps2);
//  Serial.print("\t");
*/
  tolerance = 100; // OVERRIDE TOLERANCE

  if (abs(steps_k1 - expectedSteps1) > tolerance) {
//    Serial.print(steps_k1 - expectedSteps1);
    steps_k1 = expectedSteps1;
//    steps_k1 -= 2 * (steps_k1 - expectedSteps1);
//    setMotorSpeedK1(0);
  }
  else {
//    dampen_k1 = constrain(0.01 * -(diff1 - avgDiff1), -0.01, + 0.01);
//    Serial.print(dampen_k1);
  }

  if (abs(steps_k2 - expectedSteps2) > tolerance) {
    steps_k2 = expectedSteps2;
//    setMotorSpeedK2(0);
  }

//  Serial.println();
}
