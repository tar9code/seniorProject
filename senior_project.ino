#include <Servo.h>
#include <RedBot.h>
// Arduino pins for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// 8-bit bus after the 74HC595 shift register 
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
#define SERVO1_PWM 10
#define SERVO2_PWM 9
#define SERVO3_PWM 8
#define SERVO4_PWM 7
// Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

#define LSensor 7
#define MSensor 6
#define RSensor 7
#define FrontButton 40

#define LMotor 1
#define RMotor 3
#define ClawMotor 4


#define SensorMax 1023
#define MotorSpeed 70
#define turnSpeed 100
#define MotorAdjustSpeed 50

#define DirLeft 0
#define DirRight 1

// define States of Being
#define START 0
#define FOLLOW1 1
#define TURN_RIGHT 2
#define FOLLOW2 3
#define SCORE 4
#define TURN_AROUND2 5
#define FOLLOW3 7
#define TURN_LEFT 8
#define FOLLOW4 9
#define FIND_RINGS 10
#define GRAB_RINGS 11
#define TURN_AROUND 12
// Declare classes for Servo connectors of the MotorShield.
Servo left_servo;
Servo right_servo;
Servo claw_servo;
int ledPin = 13;


//RedBotSensor left_sen = RedBotSensor(A7);
RedBotSensor middle_sen = RedBotSensor(A6);
RedBotSensor right_sen = RedBotSensor(A7);

RedBotMotors redMotor = RedBotMotors();
int lineStandard = 800;


int curDir = DirLeft;
int curState = START;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Simple Motor Shield");

  left_servo.attach(SERVO1_PWM);
  right_servo.attach(SERVO3_PWM);
  claw_servo.attach(SERVO4_PWM);
  
  pinMode(ledPin, OUTPUT);

  pinMode(FrontButton, INPUT_PULLUP);
}

void claw() {
  int cSpeed = 250;
  int cTime = 150;
  motor(ClawMotor, FORWARD, cSpeed);
  delay(cTime);
  motor(ClawMotor, BRAKE, 0);

  delay(200);

  motor(ClawMotor, BACKWARD, cSpeed);
  delay(cTime - 50);
  motor(ClawMotor, BRAKE, 0);
  delay(2000);
}

void loop() {
  //motor(LMotor, FORWARD, 100);
  //motor(RMotor, BACKWARD, 100);
  //motor(ClawMotor, FORWARD, 100);
  //claw();
  //motor(LMotor, FORWARD, 100);
  state();
  /*
  if (digitalRead(FrontButton) == LOW)
  {
    Serial.println("hit wall\n\n");
    motor(LMotor, RELEASE, 0);
    motor(RMotor, RELEASE, 0);
    delay(50000);
  }*/
}

void haltAtWall()
{
  delay(1000);
  int speed = MotorSpeed;
  motor(LMotor, FORWARD,speed);
  motor(RMotor, BACKWARD, speed);

  // while button not pressed
  while (digitalRead(FrontButton) == HIGH)
  {
    follow(0);
  }

  motor(LMotor, RELEASE, speed);
  motor(RMotor, RELEASE, speed);
}

void state()
{
  int changeState = curState; 
  switch(curState)
  {
    case START:
      delay(3000);
      Serial.print("State\n");
      changeState = FOLLOW4;
      break;
    case FOLLOW4:
      Serial.print("follow4\n");
      changeState = follow(curState);
      break;
    case FIND_RINGS:
      Serial.print("find rings\n");
      haltAtWall();
      changeState = GRAB_RINGS;
      break;
    case GRAB_RINGS:
      changeState = grabRings();
      break;
    case TURN_AROUND:
      turnAround();
      Straighten();
      motor(ClawMotor, FORWARD, 150);
      delay(50);
      motor(ClawMotor, BRAKE, 0); 
      changeState = FOLLOW1;
      break;
    case FOLLOW1:
      Serial.print("follow1\n");
      changeState = follow(curState);
      break;
    case TURN_RIGHT:
      Serial.print("turn right\n\n\n\n");
      motor(LMotor, FORWARD, 100);
      motor(RMotor, BACKWARD, 100);
      delay(30);
      motor(LMotor, BRAKE, 0);
      motor(RMotor, BRAKE, 0);
      changeState = turn(TURN_RIGHT);
      Straighten();
      //motor(LMotor, FORWARD, 100);
      delay(150);
      //motor(LMotor, BRAKE, 100);
      delay(150);
      break;
    case FOLLOW2:
      Serial.print("follow2\n");
      changeState = follow(curState);
      break;
    case SCORE:
      changeState = Score();
      break;
    case TURN_AROUND2:
      Serial.print("drop rings\n");
      // turn around 2
      turnAround();
      Straighten();
      motor(ClawMotor, FORWARD, 150);
      delay(50);
      motor(ClawMotor, BRAKE, 0); 
      changeState = FOLLOW3;
      break;
    case FOLLOW3:
      Serial.print("follow3\n");
      changeState = follow(curState);
      break;
    case TURN_LEFT:
      Serial.print("turn left\n");
      changeState = turn(TURN_LEFT);
      break;
    default:
      Serial.print("default\n");
      delay(50000);
      break;
  }

  curState = changeState;
}


int Score()
{
  Serial.print("score\n");
  motor(ClawMotor, BACKWARD, 150);
  delay(150);
  motor(ClawMotor, BRAKE, 0);
  haltAtWall();
  
  motor(LMotor, BACKWARD, 100);
  motor(RMotor, FORWARD, 100);

  delay(150);
  motor(LMotor, BRAKE, 0);
  motor(RMotor, BRAKE, 0);
  claw();
  claw();
  claw();
  return TURN_AROUND2;
}

int backUp()
{
  int out = 0;
  int mid = 0;
  int right = 0;

  motor(LMotor, BACKWARD, turnSpeed);
  motor(RMotor, FORWARD, turnSpeed);
  mid = middle_sen.read();
  right = right_sen.read();
  
  if ((right > lineStandard) && (mid > lineStandard))
  {
    out = 1;
  }
  return out;
}
void turnAround()
{
  int mid = 0;
  int back = 0;

  while (!back)
  {
    Serial.println("back up");
    back = backUp();
  }
  Serial.println("stop");
  motor(LMotor, BRAKE, turnSpeed);
  motor(RMotor, BRAKE, turnSpeed);
  delay(500);

  Serial.println("start turning around");

  motor(LMotor, FORWARD, turnSpeed * 1.6);
  motor(RMotor, FORWARD, turnSpeed * 2);
  delay(1000);
  while (mid <= lineStandard)
  {
    Serial.print("turn around\n");
    mid = middle_sen.read();
  }
  motor(LMotor, RELEASE,0);
  motor(RMotor, RELEASE, 0);
  delay(400);  
}

void Straighten()
{
  int mid = middle_sen.read();
  int right = right_sen.read();

  int diff = mid - right;

  while (diff < -100 || diff > 100)
  {
    if (diff < -100)
    {
      motor(LMotor, FORWARD, turnSpeed * 2);
      motor(RMotor, FORWARD, turnSpeed * 1.5);
    }
    else if (diff > 100)
    {
      motor(LMotor, BACKWARD, turnSpeed * 1.5);
      motor(RMotor, BACKWARD, turnSpeed * 2);
    }
    mid = middle_sen.read();
    right = right_sen.read();
    diff = mid - right;
  }
  motor(LMotor, RELEASE,0);
  motor(RMotor, RELEASE, 0);
  delay(150);
  
  /*
  motor(LMotor, FORWARD, turnSpeed * 2);
  motor(RMotor, FORWARD, turnSpeed * 1.5);
  delay(400);
  motor(LMotor, RELEASE,0);
  motor(RMotor, RELEASE, 0);
  delay(400);*/
 /* delay(150);
  motor(RMotor, FORWARD, turnSpeed);
  delay(150);
  motor(LMotor, RELEASE,0);
  motor(RMotor, RELEASE, 0);
  delay(150);*/
}


int grabRings()
{
  Serial.print("grab rings\n");
  motor(ClawMotor, BACKWARD, 150);
  delay(150);
  motor(ClawMotor, BRAKE, 0);
  return TURN_AROUND;;
}
int findRings()
{
  int wall = HIGH;
  
  motor(LMotor, RELEASE, 0);
  motor(RMotor, RELEASE, 0); 

  motor(LMotor, FORWARD, MotorSpeed / 5);
  motor(RMotor, BACKWARD, MotorSpeed / 5); 

  while (wall == HIGH)
  {
    wall = digitalRead(FrontButton);
  }

  motor(LMotor, RELEASE, 0);
  motor(RMotor, RELEASE, 0); 

  return GRAB_RINGS;
}

int turn(int currentState)
{
  int state = currentState;
  int motorSpeedLeft = turnSpeed;
  int motorSpeedRight = turnSpeed;
  Serial.println("turn method");
  delay(3000);
  int leftMotorDir = FORWARD;
  int rightMotorDir = BACKWARD;
  int sensorBorder = LSensor;
  int turnEnd = 0;
  
  if (state == TURN_RIGHT)
  {
    rightMotorDir = FORWARD;
    //motorSpeedRight = 0;
    sensorBorder = MSensor;
  }
  else if (state == TURN_LEFT)
  {
    leftMotorDir = BACKWARD;
    //motorSpeedLeft = 0;
    sensorBorder = RSensor;
  }
  else
  {
    // error
    return -2;
  }
  motor(LMotor, leftMotorDir,motorSpeedLeft * 1.6);
  motor(RMotor, rightMotorDir, motorSpeedRight * 2);
  delay(1500);
  
  while (turnEnd <= lineStandard)
  {
    Serial.print("turn\n");
    turnEnd = analogRead(sensorBorder);
  }
  motor(LMotor, RELEASE,0);
  motor(RMotor, RELEASE, 0);

  return state + 1;
}

int follow(int currentState)
{
  int mid = middle_sen.read();
  int right = right_sen.read();
  int diff = mid - right;
  int out = currentState;
  if (diff < -100 && right > lineStandard)
  {
    motor(LMotor, FORWARD, turnSpeed * 2);
    motor(RMotor, BACKWARD, MotorSpeed);
  }
  else if (diff > 100 && mid > lineStandard)
  {
    motor(LMotor, FORWARD, MotorSpeed);
    motor(RMotor, BACKWARD, turnSpeed * 2);
  }
  else if ((right > lineStandard) && (mid > lineStandard))
  {
    motor(LMotor, BRAKE, 0);
    motor(RMotor, BRAKE, 0);
    out++;
  }
  else
  {
    motor(LMotor, FORWARD, MotorSpeed);
    motor(RMotor, BACKWARD, MotorSpeed);
  }

  return out;
}

// By arduino.cc user "Krodal".
// June 2012
// Open Source / Public Domain
// everything below this point is from this open source
void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
    switch (nMotor)
    {
    case 1:
      motorA   = MOTOR1_A;
      motorB   = MOTOR1_B;
      break;
    case 2:
      motorA   = MOTOR2_A;
      motorB   = MOTOR2_B;
      break;
    case 3:
      motorA   = MOTOR3_A;
      motorB   = MOTOR3_B;
      break;
    case 4:
      motorA   = MOTOR4_A;
      motorB   = MOTOR4_B;
      break;
    default:
      break;
    }

    switch (command)
    {
    case FORWARD:
      motor_output (motorA, HIGH, speed);
      motor_output (motorB, LOW, -1);     // -1: no PWM set
      break;
    case BACKWARD:
      motor_output (motorA, LOW, speed);
      motor_output (motorB, HIGH, -1);    // -1: no PWM set
      break;
    case BRAKE:
      // The AdaFruit library didn't implement a brake.
      // The L293D motor driver ic doesn't have a good
      // brake anyway.
      // It uses transistors inside, and not mosfets.
      // Some use a software break, by using a short
      // reverse voltage.
      // This brake will try to brake, by enabling 
      // the output and by pulling both outputs to ground.
      // But it isn't a good break.
      motor_output (motorA, LOW, 255); // 255: fully on.
      motor_output (motorB, LOW, -1);  // -1: no PWM set
      break;
    case RELEASE:
      motor_output (motorA, LOW, 0);  // 0: output floating.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
    default:
      break;
    }
  }
}

void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
  case MOTOR3_A:
  case MOTOR3_B:
    motorPWM = MOTOR3_PWM;
    break;
  case MOTOR4_A:
  case MOTOR4_B:
    motorPWM = MOTOR4_PWM;
    break;
  default:
    // Use speed as error flag, -3333 = invalid output.
    speed = -3333;
    break;
  }

  if (speed != -3333)
  {
    // Set the direction with the shift register 
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but
    // not the PWM.
    shiftWrite(output, high_low);

    // set PWM only if it is valid
    if (speed >= 0 && speed <= 1000)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}

void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly, 
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}
