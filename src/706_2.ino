#include <Arduino.h>
#include <Servo.h>
#include <HCSR04.h>
#include <PID_v1.h>
#include <SharpDistSensor.h>
#include <MedianFilterLib2.h>
#include <SoftwareSerial.h>
#define BLUETOOTH_RX 10 // Serial Data input pin
#define BLUETOOTH_TX 11 // Serial Data output pin

#define motorDelay 1000

//SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

//Create Servo Objects and define digital output pins for the servo motorrs.
Servo left_font_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_font_motor;
Servo turret_motor;
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 48;
const byte right_front = 49;
const byte turret = 8;
int speed_val = 200;

//initialisation class HCSR04 (trig pin , echo pin)
HCSR04 ultrasonic(4, 5);

//initalisation class for IR Sensors
//Sen A = front left, Sen B = Front Right, Sen C = Rear Left, Sen D = Rear Right
const byte IRmedianFilterWindowSize = 1;
extern SharpDistSensor senA, senB, senC, senD;
SharpDistSensor senA(A4, IRmedianFilterWindowSize);
const float senA_C = 273479.;
const float senA_P = -1.285;
SharpDistSensor senB(A5, IRmedianFilterWindowSize);
const float senB_C = 110396;
const float senB_P = -1.128;
SharpDistSensor senC(A6, IRmedianFilterWindowSize);
const float senC_C = 38158;
const float senC_P = -1.075;
SharpDistSensor senD(A7, IRmedianFilterWindowSize);
const float senD_C = 23764;
const float senD_P = -0.999;

//set up rotation PID controller and gyro
//gyro setup
int gyroPin = A3;              //Gyro is connected to analog pin A11
float gyroVoltage = 5;         //Gyro is running at 5V
float gyroSensitivity = .007;  //Our example gyro is 7mV/deg/sec
float rotationThreshold = 1.5; //Minimum deg/sec to keep track of - helps with gyro drifting
double rotateSetpoint, rotateInput, rotateOutput;
PID rotationPID(&rotateInput, &rotateOutput, &rotateSetpoint, 5, 0.3, 0, DIRECT);

//Fan Pin
const byte fanPin = 3;

//Phototransistor Vout Pins
const byte longRange1 = A8;   //outer left PT
const byte shortRange1 = A9;  //inner left PT
const byte shortRange2 = A10; //inner right PT
const byte longRange2 = A11;  //outer right PT

//global timing variables
unsigned long searchStartTime = 0;

//number of fires extinguished
int fires = 0;

int strafedCounter = 0;
unsigned long lastTimeStrafed = 0;
int avoid_obstacles_speed = 100;
#define LEFT_LIGHT_SENSOR A8
#define RIGHT_LIGHT_SENSOR A11

//set up finite state machine
enum STATES
{
  LOCATE_FIRE,
  RELOCATE,
  AVOID_OBSTACLE,
  EXTUINGISH_FIRE,
  SHUTDOWN
};
static STATES machine_state = LOCATE_FIRE;

void setup()
{
  // BluetoothSerial.begin(115200);
  Serial.begin(9600);
  enable_motors();
  Serial.println("L1, L2, R1, R2, SONIC, WEIGHTED");
  //Set up IR Sensors
  senA.setPowerFitCoeffs(senA_C, senA_P, 0, 1023);
  senB.setPowerFitCoeffs(senB_C, senB_P, 0, 1023);
  senC.setPowerFitCoeffs(senC_C, senC_P, 50, 600);
  senD.setPowerFitCoeffs(senD_C, senD_P, 50, 600);

  //Make sure fan is turned off
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);
}

void loop()
{

  switch (machine_state)
  {

  case LOCATE_FIRE:
    machine_state = locate_fire();
    break;

  case RELOCATE:
    machine_state = relocate();
    break;

  case AVOID_OBSTACLE:
    machine_state = avoid_obstacle();
    break;

  case EXTUINGISH_FIRE:
    machine_state = extuingish_fire();
    break;

  case SHUTDOWN:
    machine_state = shutdown();
    break;
  }
}

////////////////////////////////////////////////////States/////////////////////////////////////////////////////////////////
STATES locate_fire()
{

  //initialise start search time
  searchStartTime = millis();
  strafedCounter = 0;

  while ((analogRead(A8) < 300) && (analogRead(A11) < 300))
  {
    cw(speed_val);

    //Relocate Robot if a full turn has been completed and no fire has been found;
    if (millis() - searchStartTime > 4500)
    {

      stop();
      delay(motorDelay);
      return RELOCATE;
    }
  }

  //turn cw if the right long range PT sees more light, else turn left
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

  return AVOID_OBSTACLE;

  //     int leftLightReading = analogRead(LEFT_LIGHT_SENSOR);
  //     int rightLightReading = analogRead(RIGHT_LIGHT_SENSOR);
  //     int leftLightReading2 = analogRead(A9);
  //     int rightLightReading2 = analogRead(A10);

  // leftLightReading = (leftLightReading + leftLightReading2)/2;
  // rightLightReading = (rightLightReading + rightLightReading2)/2 ;
  //     if (rightLightReading < 800 && leftLightReading < 800)
  //     {
  //       cw(150);
  //     }

  //     rightLightReading > leftLightReading ? cw(120) : ccw(120);
  //     return LOCATE_FIRE;
}

STATES relocate()
{

  unsigned long relocationTime;

  int maxDistAngle = findMaxDist();

  rotateController(maxDistAngle);
  delay(motorDelay);

  relocationTime = millis();
  //drive forward for 5 seconds
  while (((millis() - relocationTime) < 5000) && (ultrasonic.dist() > 20))
  {
    forward(speed_val);
    delay(75);
  }
  stop();
  delay(motorDelay);

  return LOCATE_FIRE;
}

STATES avoid_obstacle()
{

  int leftLightReading = analogRead(A8);
  int rightLightReading = analogRead(A11);
  int leftLightReading2 = analogRead(A9);
  int rightLightReading2 = analogRead(A10);
  float sonicReading = ultrasonic.dist();
  int turnLeftorRight = (rightLightReading * .1 + rightLightReading2 * .2 - leftLightReading * .1 - leftLightReading2 *.2);

  int distFrontLeft = senA.getDist();
  int distFrontRight = senB.getDist();
  int distLeft = senC.getDist();
  int distRight = senD.getDist();

  if (sonicReading == 0)
  {
    sonicReading = 100;
  }

  // Serial.print(rightLightReading);
  // Serial.print(", ");
  // Serial.print(rightLightReading2);
  // Serial.print(", ");
  // Serial.print(leftLightReading);
  // Serial.print(", ");
  // Serial.print(leftLightReading2);
  // Serial.print(", ");
  // Serial.print(sonicReading * 10);
  // Serial.print(", ");
  // Serial.println(turnLeftorRight);

  // forwardAndTurn(100, turnLeftorRight / 2);

  if (sonicReading < 7 && (rightLightReading2 > 200 || leftLightReading2 > 200))
  {
    unsigned long settlingTime = millis();
    while (millis() < settlingTime + 2000)
    {
      int leftLightReading = analogRead(A8);
      int rightLightReading = analogRead(A11);
      int leftLightReading2 = analogRead(A9);
      int rightLightReading2 = analogRead(A10);
      int turnLeftorRight = (rightLightReading * 1 + rightLightReading2 * .5 - leftLightReading * 1 - leftLightReading2 * .5);

      pivot(0, turnLeftorRight / 3);
      delay(20);
    }

    return EXTUINGISH_FIRE;
  }

  //check sensors values for collision

  // Serial.println(distFrontLeft);
  // Serial.println(distFrontRight);
  // Serial.println(distFront);
  // Serial.println(distLeft);
  // Serial.println(distRight);
  // Serial.println("\n\n\n\n\n");

  // if (sonicReading < 13 || distFrontRight < 150 || distFrontLeft < 150)
  // {
  //   if (((distFrontRight * .8 + distRight * .2) > (distFrontLeft * .8 + distLeft * .2)) || (((distFrontRight * .8 + distRight * .2) > 300) && ((distFrontLeft * .8 + distLeft * .2) > 300)))
  //   {
  //     strafe_right(avoid_obstacles_speed);
  //     strafedCounter++;
  //     lastTimeStrafed = millis();
  //   }
  //   else
  //   {
  //     strafe_left(avoid_obstacles_speed);
  //     strafedCounter--;
  //     lastTimeStrafed = millis();
  //   }
  // }

  else
  {
    pivot(200, turnLeftorRight/2);
  }

  if ((strafedCounter > 20 || strafedCounter < -20) && ((lastTimeStrafed + 1000) < millis()))
  {
    return LOCATE_FIRE;
  }

  return AVOID_OBSTACLE;
}

STATES extuingish_fire()
{
  digitalWrite(fanPin, HIGH);
  delay(10000);
  digitalWrite(fanPin, LOW);
  return LOCATE_FIRE;
}

STATES shutdown()
{
  return SHUTDOWN;
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
//rotates the angle given from the closest wall to face it
void rotateController(float rotateAngle)
{

  double currentAngle = 0;
  rotateSetpoint = currentAngle + rotateAngle;
  float gyroZeroVoltage = analogRead(gyroPin) * 5 / 1023.0;

  //turn the PID on
  rotationPID.SetSampleTime(10);
  rotationPID.SetOutputLimits(-speed_val, speed_val);
  rotationPID.SetMode(AUTOMATIC);

  while (abs(rotateSetpoint - currentAngle) > 10)
  {
    //This line converts the 0-1023 signal to 0-5V
    float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1023;

    //This line finds the voltage offset from sitting still
    gyroRate -= gyroZeroVoltage;

    //This line divides the voltage we found by the gyro's sensitivity
    gyroRate /= gyroSensitivity;

    //Ignore the gyro if our angular velocity does not meet our threshold
    if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold)
    {
      //This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
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
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

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
void stop() //Stop
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


  if ( turnSpeed > 0)
  {

  left_font_motor.writeMicroseconds(1500 + base_speed + turnSpeed);
  left_rear_motor.writeMicroseconds(1500 + base_speed + turnSpeed);
  right_rear_motor.writeMicroseconds(1500 - base_speed );
  right_font_motor.writeMicroseconds(1500 - base_speed );
  }

  else
  {
  left_font_motor.writeMicroseconds(1500 + base_speed );
  left_rear_motor.writeMicroseconds(1500 + base_speed );
  right_rear_motor.writeMicroseconds(1500 - base_speed - abs(turnSpeed));
  right_font_motor.writeMicroseconds(1500 - base_speed - abs(turnSpeed));


  }
}

void pivot(int base_speed, int turnSpeed)
{

  if (turnSpeed > 80)
  {
    turnSpeed = 80;
  }

  else if (turnSpeed < -80)
  {
    turnSpeed = -80;
  }


  left_font_motor.writeMicroseconds(1500 + base_speed + turnSpeed);
  left_rear_motor.writeMicroseconds(1500 + base_speed + turnSpeed);
  right_rear_motor.writeMicroseconds(1500 - base_speed + turnSpeed);
  right_font_motor.writeMicroseconds(1500 - base_speed + turnSpeed);
  


}

// void bluetoothOutput(int32_t value1, int32_t value2){
//   String Delimiter = ", ";
//   BluetoothSerial.print(value1, DEC);
//   BluetoothSerial.print(Delimiter);
//   BluetoothSerial.println(value2, DEC);
// }
