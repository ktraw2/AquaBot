/* pin definitions */
#define CLAW_MOTOR_PIN 10
#define ARM_MOTOR_PIN 11
#define LEFT_MOTOR_FORWARD_PIN 3
#define LEFT_MOTOR_BACKWARD_PIN 6
#define RIGHT_MOTOR_FORWARD_PIN 5
#define RIGHT_MOTOR_BACKWARD_PIN 9

/* angle definitions */
#define CLAW_OPEN_ANGLE 90
#define CLAW_CLOSE_ANGLE 0
#define ARM_UP_ANGLE 90
#define ARM_DOWN_ANGLE 0

/* threshold definitions */
#define WIDTH_THRESHOLD 220
#define HEIGHT_THRESHOLD 200
#define AGE_THRESHOLD 10

/* other definitions */
#define DELAY 450
#define MAX_SPEED 255
#define TARGET 0
#define SERIAL_BAUDRATE 115200
#define DEBUG
#define DEBUG_FINER

#include <Pixy2.h>
#include <Servo.h>

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
Servo armServo;
Servo clawServo;

int * targetStat = 0;
int targetStatThreshold = 0;

DC_Motor left (LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACKWARD_PIN);
DC_Motor right (RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN);


void setup() {
  // put your setup code here, to run once:
#ifdef DEBUG
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("Welcome to smarty boat!");
#endif
  
  pixy.init();
  armServo.attach(ARM_MOTOR_PIN);
  clawServo.attach(CLAW_MOTOR_PIN);
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
    
    if (age < AGE_THRESHOLD) {
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
    else {
      grabAndPlace(clawServo, armServo);
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

void grabAndPlace(Servo & claw, Servo & arm) {
  closeClaw(claw);
  delay(DELAY);
  armUp(arm);
  delay(DELAY);
  openClaw(claw);
  delay(DELAY);
  armDown(arm);
  delay(DELAY);
}

void openClaw(Servo & servo) {
  servo.write(CLAW_OPEN_ANGLE);
}

void closeClaw(Servo & servo) {
  servo.write(CLAW_CLOSE_ANGLE);
}

void armUp(Servo & servo) {
  servo.write(ARM_UP_ANGLE);
}

void armDown(Servo & servo) {
  servo.write(ARM_DOWN_ANGLE);
}
