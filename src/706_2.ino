#include <Arduino.h>
#include <Servo.h>
#include <HCSR04.h>
#include <PID_v1.h>
#include <SharpDistSensor.h>
#include <MedianFilterLib2.h>
#include <SoftwareSerial.h>

#define BLUETOOTH_RX 10 // Serial Data input pin
#define BLUETOOTH_TX 11 // Serial Data output pin
#define TURN_SATURATE 100
#define motorDelay 1000
#define SENSORS_FRONT 2800       // turret microseconds reading to face sensors forward
#define SENSORS_BACK 500         // turrent ... face sensors backwards
#define SIDE_SCAN_RESOLUTION 6   // resoulition of scan values to take
#define SIDE_SCAN_MAX 130        // max scan value that it deems room to move that way
#define STRAFE_TIME_CONSTANT 650 // constant strafe(200) duration after a side scan
#define RESCAN_COOLDOWN 1300     // time after a scan that it doesnt scan again
#define MIN_STRAFE_READING 240   // decrease this number, and it increases the distance between car and obstacle as it drives around it. OBSTACLE GETS CLOSER -> READING GETS HIGHER
#define MAX_STRAFE_READING 375   // same
// SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Create Servo Objects and define digital output pins for the servo motorrs.
bool recentlyScanned = 0;
double timeSinceScanned = 0;

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 0, 0, DIRECT);

Servo left_font_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_font_motor;
Servo turret_motor;
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 48;
const byte right_front = 49;
const byte turret = 33;
int speed_val = 200;
float K_P = 2.0;

int fireFlag = 0; //

int leftDistReading;
int rightDistReading;

// initialisation class HCSR04 (trig pin , echo pin)
HCSR04 ultrasonic(4, 5);

// initalisation class for IR Sensors
// Sen A = front right, Sen B = Front left, Sen C = Rear Left, Sen D = Rear Right
const byte IRmedianFilterWindowSize = 5;
extern SharpDistSensor senA, senB, senC, senD;
SharpDistSensor senA(A5, IRmedianFilterWindowSize);
const float senA_C = 273479.;
const float senA_P = -1.285;
SharpDistSensor senB(A4, IRmedianFilterWindowSize);
const float senB_C = 110396;
const float senB_P = -1.128;
SharpDistSensor senC(A6, 5);
const float senC_C = 38158;
const float senC_P = -1.075;
SharpDistSensor senD(A7, 5);
const float senD_C = 23764;
const float senD_P = -0.999;

// set up rotation PID controller and gyro
// gyro setup
int gyroPin = A3;              // Gyro is connected to analog pin A11
float gyroVoltage = 5;         // Gyro is running at 5V
float gyroSensitivity = .007;  // Our example gyro is 7mV/deg/sec
float rotationThreshold = 1.5; // Minimum deg/sec to keep track of - helps with gyro drifting
double rotateSetpoint, rotateInput, rotateOutput;
PID rotationPID(&rotateInput, &rotateOutput, &rotateSetpoint, 5, 0.3, 0, DIRECT);

// Fan Pin
const byte fanPin = 3;

// Phototransistor Vout Pins
const byte longRangeLeft = A8;    // outer left PT
const byte shortRangeLeft = A9;   // inner left PT
const byte shortRangeRight = A10; // inner right PT
const byte longRangeRight = A11;  // outer right PT3

// global timing variables
unsigned long searchStartTime = 0;

// number of fires extinguished
int fires = 0;

int strafedCounter = 0;
unsigned long lastTimeStrafed = 0;
int avoid_obstacles_speed = 100;
#define LEFT_LIGHT_SENSOR A8
#define RIGHT_LIGHT_SENSOR A11

// set up finite state machine
enum STATES
{
  LOCATE_FIRE,
  //RELOCATE,
  HEAD_TOWARDS_FIRE,
  AVOID_OBSTACLE_RIGHT,
  NO_SCAN_AVOID_OBSTACLE_RIGHT,
  NO_SCAN_AVOID_OBSTACLE_LEFT,
  AVOID_OBSTACLE_LEFT,
  EXTUINGISH_FIRE,
  SHUTDOWN,
  DEBUG
};
static STATES machine_state = LOCATE_FIRE;

void setup()
{
  // BluetoothSerial.begin(115200);
  Serial.begin(9600);
  enable_motors();
  // Serial.println("L1, L2, R1, R2, SONIC, WEIGHTED");
  // Set up IR Sensors
  senA.setPowerFitCoeffs(senA_C, senA_P, 0, 1023);
  senB.setPowerFitCoeffs(senB_C, senB_P, 0, 1023);
  senC.setPowerFitCoeffs(senC_C, senC_P, 50, 600);
  senD.setPowerFitCoeffs(senD_C, senD_P, 50, 600);
  turret_motor.writeMicroseconds(SENSORS_FRONT);
  // Make sure fan is turned off
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);
  // Serial.println("distLeft,DistRight");
}

void loop()
{

  switch (machine_state)
  {

  case LOCATE_FIRE:
    machine_state = locate_fire();
    break;

  // case RELOCATE:
  //   machine_state = relocate();
  //   break;

  case HEAD_TOWARDS_FIRE:
    machine_state = head_toward_fire();
    break;

  case AVOID_OBSTACLE_LEFT:
    machine_state = avoid_obstacle_left();
    break;

  case AVOID_OBSTACLE_RIGHT:
    machine_state = avoid_obstacle_right();
    break;

  case NO_SCAN_AVOID_OBSTACLE_LEFT:
    machine_state = no_scan_avoid_obstacle_left();
    break;

  case NO_SCAN_AVOID_OBSTACLE_RIGHT:
    machine_state = no_scan_avoid_obstacle_right();
    break;

  case EXTUINGISH_FIRE:
    machine_state = extuingish_fire();
    break;

  case SHUTDOWN:
    machine_state = shutdown();
    break;

  case DEBUG:
    machine_state = debug();
    break;
  }
}

////////////////////////////////////////////////////States/////////////////////////////////////////////////////////////////
STATES locate_fire()
{

  // initialise start search time
  searchStartTime = millis();
  strafedCounter = 0;
  int lightLimit = 300;
  while ((analogRead(A8) < lightLimit) && (analogRead(A11) < lightLimit))

  {
    cw(speed_val);

    // Relocate Robot if a full turn has been completed and no fire has been found;
    if (millis() - searchStartTime > 4500)
    {

      lightLimit = lightLimit - 50;
      searchStartTime = millis();
    }
  }

  // turn cw if the right long range PT sees more light, else turn left
  if ((analogRead(A11) - analogRead(A8)) > 20)
  {
    while ((analogRead(A11) - analogRead(A8)) > 20)
    {
      cw(speed_val);
    }
  }
  else
  {
    while ((analogRead(A8) - analogRead(A11)) > 20)
    {
      ccw(speed_val);
    }
  }
  stop();
  delay(motorDelay);

  return HEAD_TOWARDS_FIRE;
}

// STATES relocate()
// {

//   unsigned long relocationTime;

//   int maxDistAngle = findMaxDist();

//   rotateController(maxDistAngle);
//   delay(motorDelay);

//   relocationTime = millis();

//   int IFRLeft = analogRead(A4);
//   int IFRRight = analogRead(A5);

//   // drive forward for 5 seconds
//   while (((millis() - relocationTime) < 2000) && (ultrasonic.dist() > 20) && !(IFRLeft < MAX_STRAFE_READING && IFRLeft > MIN_STRAFE_READING) && !(IFRRight < MAX_STRAFE_READING && IFRRight > MIN_STRAFE_READING)))
//   {
//     forward(speed_val);
//     IFRLeft = analogRead(A4);
//     IFRRight = analogRead(A5);
//     delay(40);
//   }
//   stop();
//   delay(motorDelay);

//   return LOCATE_FIRE;
// }

STATES head_toward_fire()
{
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-150,150);
  myPID.SetSampleTime(40);
  myPID.SetTunings(2, 0, 0);

  while (1)
  {

    if (timeSinceScanned + RESCAN_COOLDOWN < millis())
    {
      recentlyScanned = 0;
    }

    int longRangeLeftReading = analogRead(longRangeLeft);
    int longRangeRightReading = analogRead(longRangeRight);
    int shortRangeLeftReading = analogRead(shortRangeLeft);
    int shortRangeRightReading = analogRead(shortRangeRight);

    int IFRLeft = analogRead(A4);
    int IFRRight = analogRead(A5);
    int IFRrearLeft = senC.getDist();
    int IFRrearRight = senD.getDist();

    float sonicReading = ultrasonic.dist();
    int controlError = (longRangeRightReading * .3 + 1 * shortRangeRightReading - longRangeLeftReading * .3 - 1 * shortRangeLeftReading);
    int speed;
    Input = -controlError;

    if (sonicReading == 0)
    {
      sonicReading = 100;
    }

    if ((sonicReading < 20) && ((shortRangeRightReading < 100) && (shortRangeLeftReading < 100))) // if sonicReading less than 20cm,  and short range sensors are somthing..
    {
      speed = 100;
    }
    else
    {
      speed = 200;
    }

    myPID.Compute();
    pivot(speed, Output); // P controller which tried to always head towards a light.

    // detect if there is an obstacle to the right.
    if ((IFRLeft < MAX_STRAFE_READING && IFRLeft > MIN_STRAFE_READING) && ((shortRangeRightReading < 100) && (shortRangeLeftReading < 100)))
    {
      stop();
      delay(200);

      if (recentlyScanned)
      {
        return NO_SCAN_AVOID_OBSTACLE_RIGHT;
      }

      return AVOID_OBSTACLE_RIGHT;
    }

    // detect if there is an obstacle to the left.
    if ((IFRRight < MAX_STRAFE_READING && IFRRight > MIN_STRAFE_READING) && ((shortRangeRightReading < 100) && (shortRangeLeftReading < 100)))
    {
      stop();
      delay(200);

      if (recentlyScanned)
      {
        return NO_SCAN_AVOID_OBSTACLE_LEFT;
      }

      return AVOID_OBSTACLE_LEFT;
    }

    if (sonicReading < 10 && ((shortRangeLeftReading > 200 || shortRangeRightReading > 200)))
    {
      myPID.SetMode(MANUAL);
      stop();
      delay(500);
      return EXTUINGISH_FIRE;
    }

    delay(20);

    return HEAD_TOWARDS_FIRE;
  }
}

STATES avoid_obstacle_right()
{

  scanTurret();


  if (leftDistReading < SIDE_SCAN_MAX)
  {
    strafe_right(200);
    delay(STRAFE_TIME_CONSTANT);
    stop();
    return HEAD_TOWARDS_FIRE;
  }

  while (analogRead(A4) > MIN_STRAFE_READING)

  {

    strafe_left(200);
    delay(50);
  }

  stop();
  return HEAD_TOWARDS_FIRE;
}

STATES avoid_obstacle_left()
{

  scanTurret();

  if (rightDistReading < SIDE_SCAN_MAX)
  {

    turret_motor.writeMicroseconds(1600);
    while(rightDistReading < SIDE_SCAN_MAX && leftDistReading < SIDE_SCAN_MAX)
    
    strafe_left(200);
    delay(STRAFE_TIME_CONSTANT*1.2);
    stop();
    return HEAD_TOWARDS_FIRE;
  }

  while (analogRead(A5) > MIN_STRAFE_READING)

  {

    strafe_right(200);
    delay(50);
  }

  stop();
  return HEAD_TOWARDS_FIRE;
}

STATES no_scan_avoid_obstacle_right()
{

  int IFRLeft = analogRead(A4);

  strafe_left(200);
  if (IFRLeft < MIN_STRAFE_READING)
  {
    stop();
    return HEAD_TOWARDS_FIRE;
  }
  delay(50);
  return NO_SCAN_AVOID_OBSTACLE_RIGHT;
}

STATES no_scan_avoid_obstacle_left()
{

  int IFRRight = analogRead(A5);

  strafe_right(200);
  if (IFRRight < 250)
  {
    stop();
    return HEAD_TOWARDS_FIRE;
  }
  delay(50);
  return NO_SCAN_AVOID_OBSTACLE_LEFT;
}

STATES extuingish_fire()
{
  myPID.SetMode(AUTOMATIC);
  int shortRangeLeftReading = analogRead(shortRangeLeft);
  int shortRangeRightReading = analogRead(shortRangeRight);
  int longRangeLeftReading = analogRead(longRangeLeft);
  int longRangeRightReading = analogRead(longRangeRight);

  myPID.SetTunings(.2, .1, .1);
  digitalWrite(fanPin, HIGH);
  myPID.SetSampleTime(5);
  while (longRangeLeftReading > 800 || longRangeRightReading > 800) // while shortrange sensors see a fire infront
  {

    int controlError = ( shortRangeRightReading -  shortRangeLeftReading);
    Input =  -controlError;
    
    myPID.Compute();
    // if abs(Output) < 
    pivot(0, Output);
    shortRangeLeftReading = analogRead(shortRangeLeft);
    shortRangeRightReading = analogRead(shortRangeRight);
    longRangeLeftReading = analogRead(longRangeLeft);
    longRangeRightReading = analogRead(longRangeRight);
  }
  myPID.SetMode(MANUAL);
  digitalWrite(fanPin, LOW); // fire not anymore detected -> move on.
  fireFlag++;

  if (fireFlag < 2)
  {
    reverse(150); // go in reverse a bit
    delay(500);   // wait a second
    stop();       // stop
    delay(100);   // wait a bit
  }

  if (fireFlag == 2)
  {
    return SHUTDOWN;
  }

  return LOCATE_FIRE;
}

STATES shutdown()
{

  disable_motors();
  delay(5000);
  digitalWrite(13, HIGH); // flash inbuilt LED
  delay(5000);
  digitalWrite(13, LOW);
  return SHUTDOWN;
}

STATES debug()
{
  scanTurret();
  return DEBUG;
}

//////////////////////////////////////////////////////////Relocate to max distance/////////////////////////////////
int findMaxDist(void)
{

  signed int maxAngle = 0;
  float maxDistance = ultrasonic.dist();
  float tempDistance;

  for (int i = 1; i <= 7; i++)
  {
    rotateController(45);
    delay(motorDelay);

    tempDistance = ultrasonic.dist();

    if (tempDistance > maxDistance)
    {
      maxDistance = tempDistance;
      maxAngle = i * 45;
    }
  }

  return maxAngle + 45;
}

/////////////////////////////////////////////////////////PID Rotate Function///////////////////////////////////////
// rotates the angle given from the closest wall to face it
void rotateController(float rotateAngle)
{

  double currentAngle = 0;
  rotateSetpoint = currentAngle + rotateAngle;
  float gyroZeroVoltage = analogRead(gyroPin) * 5 / 1023.0;

  // turn the PID on
  rotationPID.SetSampleTime(10);
  rotationPID.SetOutputLimits(-speed_val, speed_val);
  rotationPID.SetMode(AUTOMATIC);

  while (abs(rotateSetpoint - currentAngle) > 10)
  {
    // This line converts the 0-1023 signal to 0-5V
    float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1023;

    // This line finds the voltage offset from sitting still
    gyroRate -= gyroZeroVoltage;

    // This line divides the voltage we found by the gyro's sensitivity
    gyroRate /= gyroSensitivity;

    // Ignore the gyro if our angular velocity does not meet our threshold
    if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold)
    {
      // This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
      gyroRate /= 100;
      currentAngle += gyroRate;
    }

    rotateInput = currentAngle;
    rotationPID.Compute();
    rotatePID(rotateOutput);
    delay(10);
  }
  stop();
}

//----------------------Motor moments------------------------
// The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach(); // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach(); // detach the servo on pin right_front to turn Vex Motor Controller 29 Off
  turret_motor.detach();

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
  pinMode(turret, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);   // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);    // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front); // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
  turret_motor.attach(turret);
}
void stop() // Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward(int speed_val)
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse(int speed_val)
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw(int speed_val)
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw(int speed_val)
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left(int speed_val)
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right(int speed_val)
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void rotatePID(int Output)
{
  left_font_motor.writeMicroseconds(1500 + Output);
  left_rear_motor.writeMicroseconds(1500 + Output);
  right_rear_motor.writeMicroseconds(1500 + Output);
  right_font_motor.writeMicroseconds(1500 + Output);
}

void forwardAndTurn(int base_speed, int turnSpeed)
{

  // if (turnSpeed > 200)
  // {
  //   turnSpeed = 200;
  // }

  // else if (turnSpeed < -200)
  // {
  //   turnSpeed = -200;
  // }

  if (turnSpeed > 0)
  {

    left_font_motor.writeMicroseconds(1500 + base_speed + turnSpeed);
    left_rear_motor.writeMicroseconds(1500 + base_speed + turnSpeed);
    right_rear_motor.writeMicroseconds(1500 - base_speed);
    right_font_motor.writeMicroseconds(1500 - base_speed);
  }

  else
  {
    left_font_motor.writeMicroseconds(1500 + base_speed);
    left_rear_motor.writeMicroseconds(1500 + base_speed);
    right_rear_motor.writeMicroseconds(1500 - base_speed - abs(turnSpeed));
    right_font_motor.writeMicroseconds(1500 - base_speed - abs(turnSpeed));
  }
}

void pivot(int base_speed, int turnSpeed)
{

  if (turnSpeed > TURN_SATURATE) // Limit turnSpeed to saturation value (hardcoded value)
  {
    turnSpeed = TURN_SATURATE;
  }

  else if (turnSpeed < -TURN_SATURATE) // Limit turnSpeed to saturation value (hardcoded value)
  {
    turnSpeed = -TURN_SATURATE;
  }

  base_speed = base_speed - abs(turnSpeed); // if turnspeed is large, base_speed decreases to give time to turn

  if (base_speed < 0) // base speed shouldnt be less than 0 however.
  {
    base_speed = 0;
  }

  left_font_motor.writeMicroseconds(1500 + base_speed + turnSpeed);
  left_rear_motor.writeMicroseconds(1500 + base_speed + turnSpeed);
  right_rear_motor.writeMicroseconds(1500 - base_speed + turnSpeed);
  right_font_motor.writeMicroseconds(1500 - base_speed + turnSpeed);
}

void scanTurret()

{
  leftDistReading = 999;
  rightDistReading = 999;

  for (int i = 0; i < SIDE_SCAN_RESOLUTION + 1; i++)
  {

    turret_motor.writeMicroseconds(SENSORS_FRONT - (SENSORS_FRONT - SENSORS_BACK) / SIDE_SCAN_RESOLUTION * i);
    delay(150);
    for (int j = 0; j < 4; j++)
    {
      senD.getDist();
      senC.getDist();
      delay(30);
    }

    int leftDistReadingCurrent = senC.getDist();
    int rightDistReadingCurrent = senD.getDist();
    delay(50);
    if (leftDistReadingCurrent < leftDistReading)
    {
      leftDistReading = leftDistReadingCurrent;
    }

    if (rightDistReadingCurrent < rightDistReading)
    {
      rightDistReading = rightDistReadingCurrent;
    }
  }

  recentlyScanned = 1;
  timeSinceScanned = millis();
  turret_motor.writeMicroseconds(SENSORS_FRONT);
}