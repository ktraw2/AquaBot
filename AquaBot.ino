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
#define CLAW_SERVO_NUM 3
#define ARM_SERVO_NUM 7

/* angle definitions */
#define CLAW_OPEN_ANGLE 90
#define CLAW_CLOSE_ANGLE 0
#define ARM_UP_ANGLE 90
#define ARM_DOWN_ANGLE 0

/* threshold definitions */
#define WIDTH_THRESHOLD 175 /* old: 220 */
#define HEIGHT_THRESHOLD 150 /* old: 200 */
#define AGE_MIN_THRESHOLD 50
#define AGE_MAX_THRESHOLD 60

/* other definitions */
#define DELAY 450
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
      analogWrite(forward_pin, intensity);
    }
    else {
#ifdef DEBUG_FINER
      Serial.print("Backward on: ");
      Serial.println(backward_pin);
#endif
      analogWrite(backward_pin, intensity);
    }
  }

  void off() {
    analogWrite(forward_pin, 0);
    analogWrite(backward_pin, 0);
  }

  void drive(byte intensity, bool forward, int time) {
    on(intensity, forward);
    delay(time);
    off();
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
  
  pixy.init();
  servoController.begin();
  servoController.setPWMFreq(SERVO_FREQUENCY);
}

void loop() {
  // put your main code here, to run repeatedly:
  pixy.ccc.getBlocks();

#ifdef DEBUG
  Serial.println("");
  Serial.print("Number of objects: ");
  Serial.println(pixy.ccc.numBlocks);
#endif
  
  if(pixy.ccc.numBlocks) {
    /* steps:
     *  1. Turn iteratively to face the detected object (a little at a time)
     *     (aka x and y of detected object should be in center)
     *  2. Move iteratively forward towards the detected object
     *     (aka width/height of detectec object should be increasing)
     *  3. Call grabAndPlace
     *  4. Wait for a new object
     */
    const int & age = pixy.ccc.blocks[TARGET].m_age;
    const int & height = pixy.ccc.blocks[TARGET].m_height;
    const int & width = pixy.ccc.blocks[TARGET].m_width;

#ifdef DEBUG
    Serial.print("Age: ");
    Serial.println(age);
    Serial.print("Height: ");
    Serial.println(height);
    Serial.print("Width: ");
    Serial.println(width);
#endif
    
    if (age > AGE_MIN_THRESHOLD && age < AGE_MAX_THRESHOLD) {
      goneReverse = false;
      if (width >= height) {
#ifdef DEBUG
      Serial.println("Want to increase: height");
#endif
        /* we want height to increase */
        targetStat = &height;
        targetStatThreshold = HEIGHT_THRESHOLD;
      }
      else {
#ifdef DEBUG
      Serial.println("Want to increase: width");
#endif 
        /* we want width to increase */
        targetStat = &width;
        targetStatThreshold = WIDTH_THRESHOLD;
      }
    }
    else if (age < AGE_MIN_THRESHOLD) {
      targetStat = NULL;
    }

     
    
    if (targetStat && (*targetStat < targetStatThreshold)) {
       //int speed = MAX_SPEED - (((height + width) / 2) % MAX_SPEED);
       int speed = MAX_SPEED - ((targetStatThreshold + *targetStat) % MAX_SPEED);

#ifdef DEBUG
      Serial.print("Speed: ");
      Serial.println(speed);
#endif
       
       driveTwo(left, right, speed, true, DELAY);
       return;
    }
    else if (targetStat && (*targetStat >= targetStatThreshold)){
      if (goneReverse == false) {
        delay(DELAY);
        driveTwo(left, right, REVERSE_SPEED, false, DELAY);
        goneReverse = true;
      }
      else {
        grabAndPlace(servoController);
      }
      return;
    }
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

void grabAndPlace(Adafruit_PWMServoDriver & controller) {
  closeClaw(controller);
  delay(DELAY);
  armUp(controller);
  delay(DELAY);
  openClaw(controller);
  delay(DELAY);
  armDown(controller);
  delay(DELAY);
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
