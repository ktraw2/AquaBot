/* pin definitions for DC motors */
#define LEFT_MOTOR_FORWARD_PIN 6// 3
#define LEFT_MOTOR_BACKWARD_PIN 3 // 6
#define RIGHT_MOTOR_FORWARD_PIN 5 // 5 good
#define RIGHT_MOTOR_BACKWARD_PIN 9 // 11

/* servo constants */
#define SERVO_MIN 75
#define SERVO_MAX 650
#define SERVO_MIN_DEGREES 0
#define SERVO_MAX_DEGREES 270
#define SERVO_FREQUENCY 60
#define CLAW_SERVO_NUM 2
#define ARM_SERVO_NUM 6

/* angle definitions */
#define CLAW_OPEN_ANGLE 10
#define CLAW_CLOSE_ANGLE 35
#define ARM_UP_ANGLE 25
#define ARM_DOWN_ANGLE 120

/* threshold definitions */
#define WIDTH_THRESHOLD 175 /* old: 220 */
#define HEIGHT_THRESHOLD 150 /* old: 200 */
#define AGE_MIN_THRESHOLD 50
#define AGE_MAX_THRESHOLD 60

/* other definitions */
#define MAX_DELTA_POWER 250
#define DELAY 450
#define GRAB_PLACE_DELAY 2000
#define MAX_SPEED 200
#define REVERSE_SPEED 255
#define TARGET 0
#define SERIAL_BAUDRATE 115200
#define DEBUG

#include <Pixy2.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* define a DC motor */
class DC_Motor {
  const byte forward_pin;
  const byte backward_pin;

public:
  DC_Motor(byte forwardPin, byte backwardPin):
  forward_pin(forwardPin), backward_pin(backwardPin) {
    pinMode(forward_pin, OUTPUT);
    pinMode(backward_pin, OUTPUT);
  }

  void on(byte intensity, bool forward) {
    if (forward) {
#ifdef DEBUG_FINER
      Serial.print("Forward on: ");
      Serial.println(forward_pin);
#endif
      analogWrite(backward_pin, 0);
      analogWrite(forward_pin, intensity);
      delay(40);
    }
    else {
#ifdef DEBUG_FINER
      Serial.print("Backward on: ");
      Serial.println(backward_pin);
#endif
      analogWrite(forward_pin , 0);
      analogWrite(backward_pin, intensity);
      delay(40);
    }
  }

  void off() {
    analogWrite(forward_pin, 0);
    analogWrite(backward_pin, 0);
  }

  void drive(byte intensity, bool forward, int time) {
    on(intensity, forward);
    //delay(time);
    //off();
  }
};

Pixy2 pixy;
Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver();

int * targetStat = 0;
int targetStatThreshold = 0;

DC_Motor left (LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACKWARD_PIN);
DC_Motor right (RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN);

bool goneReverse = false;


void setup() {
  
  // put your setup code here, to run once:
#ifdef DEBUG
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("Welcome to smarty boat!");
#endif

#ifdef DEBUG
  uint8_t returnVal = pixy.init();
  Serial.print("Pixy init returned: ");
  Serial.println(returnVal);
#else
  pixy.init();
#endif
  
  servoController.begin();
  servoController.setPWMFreq(SERVO_FREQUENCY);
  openClaw(servoController);
  armDown(servoController);
}



int leftSpeedtoMotor = 0; // control to motors
int rightSpeedtoMotor = 0;

int leftSp = 0;//default forward drive: turn in circles
int rightSp = 0;

int   leftp = 0, rightp = 0;
int serror, slasterror = 0, ssumerror = 0;
float sP = 10; //speed pid gains(back and forth) 10
float sI = 0;
float sD = 50; //50

int Speed, Turn, rerror, rlasterror = 0, rsumerror = 0;
float rP = 10;//angular pid constants(rotate) 7
float rI = 15;//15
float rD = 100;//100
  int leftslew=0, rightslew=0;
int errorAge = 0;

void loop() {
  // put your main code here, to run repeatedly:
  pixy.ccc.getBlocks();

#ifdef DEBUG
  Serial.println("");
  Serial.print("Number of objects: ");
  Serial.println(pixy.ccc.numBlocks);
#endif
  
  if(pixy.ccc.numBlocks) {

    CalcError();
    Speed = serror*sP + ssumerror*sI + (serror - slasterror)*sD;
    Turn = rerror*rP + rsumerror*rI + (rerror - rlasterror)*rD;
// memory turn necessary?

    ssumerror = ssumerror + serror;
    rsumerror = rsumerror + rerror;
    
    if (ssumerror > 15) {ssumerror = 15;} // prevents integrator wind-up
    else if (ssumerror < -15) {ssumerror = -15;}
  
    if (rsumerror > 15) {rsumerror = 15;} 
    else if (rsumerror < -15) {rsumerror = -15;}
  
    slasterror = serror;
    rlasterror = rerror;
  
    if      (Turn < 0) {leftp = Turn ; rightp = -Turn;}   //slow down one motor, increase other motor
    else if (Turn > 0) {leftp = Turn ; rightp = -Turn;} 
    else               {leftp = 0    ; rightp = 0;}
    
    //run the motors
    leftSpeedtoMotor = min(leftSp+Speed+leftp,250); // limits speed to MAX_SPEED   leftSp+Speed+leftp,200
    rightSpeedtoMotor = min(rightSp+Speed+rightp,250); // (leftSp & rightSp are default speeds)
/*
    if(leftslew - leftSpeedtoMotor > MAX_DELTA_POWER) // slew motors to decrease spike
      { //motor trying to lose power or reverse direction
        leftSpeedtoMotor= leftslew - MAX_DELTA_POWER;
        rightSpeedtoMotor= rightslew - MAX_DELTA_POWER;
      }
    if(leftslew - leftSpeedtoMotor < MAX_DELTA_POWER)
      { //motor trying to gain power or reverse direction
        leftSpeedtoMotor = leftslew + MAX_DELTA_POWER;
        rightSpeedtoMotor= rightslew + MAX_DELTA_POWER;
      }
      leftslew = leftSpeedtoMotor;
    rightslew = rightSpeedtoMotor;    
      */
    if (leftSpeedtoMotor > 0){
      left.on(leftSpeedtoMotor, true);
    }
    else{
      leftSpeedtoMotor = max(leftSp+Speed+leftp,-250);//-100 min?
      left.on(-leftSpeedtoMotor, false);
    }
    if (rightSpeedtoMotor > 0){
      right.on(rightSpeedtoMotor, true);
    }
    else{
      rightSpeedtoMotor = max(rightSp+Speed+rightp,-250);
      right.on(-rightSpeedtoMotor, false);
    }
  }else{
    leftp = 0;
    rightp = 0;
    slasterror = 0; ssumerror = 0;
    rlasterror = 0; rsumerror = 0;
    left.off();
    right.off();
  }
}

void driveTwo(DC_Motor & one, DC_Motor & two, byte intensity, bool forward, int time) {
#ifdef DEBUG_FINER
  Serial.println("Turning two motors on...");
#endif
  two.on(intensity, forward);
  one.on(intensity, forward);
  
  delay(time);
#ifdef DEBUG_FINER
  Serial.println("Turning two motors off...");
#endif
  one.off();
  two.off();
}

// calculate angle error(rerror) and distance error(serror) from the pixycam
void CalcError(){
  
   const int & age = pixy.ccc.blocks[TARGET].m_age;
   //const int & height = pixy.ccc.blocks[TARGET].m_height;
   const int & width = pixy.ccc.blocks[TARGET].m_width;
   const int & x = pixy.ccc.blocks[TARGET].m_x;
  //pixy.blocks[i].signature The signature number of the detected object (1-7)
  //pixy.blocks[i].x The x location of the center of the detected object (0 to 319)
  //pixy.blocks[i].y The y location of the center of the detected object (0 to 199)
  //pixy.blocks[i].width The width of the detected object (1 to 320)
  //pixy.blocks[i].height The height of the detected object (1 to 200)
  //pixy.blocks[i].print() A member function that prints the detected object information to the serial port
   rerror = x-160; //160 is center of vision
   #ifdef DEBUG
    Serial.print("rerror: ");
    Serial.println(rerror);
   #endif
   serror = 310 - width; //width_THRESHOLD
   #ifdef DEBUG
    Serial.print("serror: ");
    Serial.println(serror);
   #endif

   if(abs(rerror) < 50 && abs(serror) < 50){ //acceptable error
      errorAge++;
   }else { errorAge = 0; }
   if(errorAge > 25){ //hysteresis
      left.off();
      right.off();
      grabAndPlace(servoController);
    #ifdef DEBUG
    Serial.println("called: ");
    //Serial.println();
   #endif
   delay(1000);
   }
   
}  // end CalcError()

void grabAndPlace(Adafruit_PWMServoDriver & controller) {
  closeClaw(controller);
  delay(GRAB_PLACE_DELAY);
  armUp(controller);
  delay(GRAB_PLACE_DELAY);
  openClaw(controller);
  delay(GRAB_PLACE_DELAY);
  armDown(controller);
  delay(GRAB_PLACE_DELAY);
}

void openClaw(Adafruit_PWMServoDriver & controller) {
  uint16_t pulselen = map(CLAW_OPEN_ANGLE, SERVO_MIN_DEGREES, SERVO_MAX_DEGREES, SERVO_MIN, SERVO_MAX);
  controller.setPWM(CLAW_SERVO_NUM, 0, pulselen);
}

void closeClaw(Adafruit_PWMServoDriver & controller) {
  uint16_t pulselen = map(CLAW_CLOSE_ANGLE, SERVO_MIN_DEGREES, SERVO_MAX_DEGREES, SERVO_MIN, SERVO_MAX);
  controller.setPWM(CLAW_SERVO_NUM, 0, pulselen);
}

void armUp(Adafruit_PWMServoDriver & controller) {
  uint16_t pulselen = map(ARM_UP_ANGLE, SERVO_MIN_DEGREES, SERVO_MAX_DEGREES, SERVO_MIN, SERVO_MAX);
  controller.setPWM(ARM_SERVO_NUM, 0, pulselen);
}

void armDown(Adafruit_PWMServoDriver & controller) {
  uint16_t pulselen = map(ARM_DOWN_ANGLE, SERVO_MIN_DEGREES, SERVO_MAX_DEGREES, SERVO_MIN, SERVO_MAX);
  controller.setPWM(ARM_SERVO_NUM, 0, pulselen);
}
