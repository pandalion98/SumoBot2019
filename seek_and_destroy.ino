#include<NewPing.h>
#define BATTLE_DEBUG false

// CONTROLS!!!!!!1!
#define FORWARD   1
#define BACKWARD  2
#define BRAKEGND 3
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#define STARTBTN 26

// ---------- VNH2SP30 pin definitions ----------
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
// ---------- VNH2SP30 pin definitions ----------

// ---------- IR sensors ----------
int IR_ANALOG_CALIB = 3500; // IR sensor analogRead value when placed in black area
#define IR_SAMPLES_PER_STEP 3 // Number of samples for data averaging. See code.
#define IR_CALIBRATION_SAMPLES 10
#define IRFL 0 // IR, Front left
#define IRFR 1
#define IRBL 2
#define IRBR 3

int irAnalogPins[4] = {A8, A9, A11, A10};
// ---------- IR sensors ----------

bool enemyTargeted = false;
bool rescueMode = false;
bool battleTrig = false;

void setup()
{
  if (BATTLE_DEBUG) {
    Serial.begin(250000);
  }

  pinMode(STARTBTN, INPUT_PULLUP);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }

  for (int i = 0; i < 4; i++)
  {
    pinMode(irAnalogPins[i], INPUT);
  }
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }

  // Wait for button press.
  if (!battleTrig) {
    awaitStart();
  }

  // Floor calibration procedure. We're in black, and everything else is white.
  // LPU #NeverAgain
  calibrateBlack();
}

void loop()
{

  checkSurroundings();
}

void checkSurroundings() {
  // Read average, so that we don't consider spurious data (e.g. area that's too shiny)
  // The more reflected, the lesser the value.
  int aveFL = 9999; // Front left, fLeft
  int aveFR = 9999;
  int aveBL = 9999;
  int aveBR = 9999; // Back right, bRight

  for (int i = 0; i < IR_SAMPLES_PER_STEP; i++) {
    aveFL += analogRead(irAnalogPins[IRFL]);
    aveFR += analogRead(irAnalogPins[IRFR]);
    aveBL += analogRead(irAnalogPins[IRBL]);
    aveBR += analogRead(irAnalogPins[IRBR]);
  }

  aveFL = (int) (aveFL / IR_SAMPLES_PER_STEP);
  aveFR = (int) (aveFR / IR_SAMPLES_PER_STEP);
  aveBL = (int) (aveBL / IR_SAMPLES_PER_STEP);
  aveBR = (int) (aveBR / IR_SAMPLES_PER_STEP);

  if (BATTLE_DEBUG) {
    char data[100];
    sprintf(data, "FL %d FR %d BL %d BR %d", aveFL, aveFR, aveBL, aveBR);
    Serial.println(data);
  }

  // We can compress this part somewhere, but we don't have time
  bool fLeftWhite = aveFL < IR_ANALOG_CALIB;
  bool fRightWhite = aveFR < IR_ANALOG_CALIB;
  bool bLeftWhite = aveBL < IR_ANALOG_CALIB;
  bool bRightWhite = aveBR < IR_ANALOG_CALIB;

  int leftWhSpd = random(127, 255);
  int rightWhSpd = random(127, 255);

  // Decision tree. What to do if IR detects we're in white.
  if (fLeftWhite || fRightWhite) {         // Whole FRONT face in white
    rescueMode = true;
    motorGo(MOTOR_LEFT, BACKWARD, leftWhSpd);
    motorGo(MOTOR_RIGHT, BACKWARD, rightWhSpd);
    // Full reverse for X ms
  } else if (bLeftWhite || bRightWhite) {
    rescueMode = true;
    motorGo(MOTOR_LEFT, FORWARD, leftWhSpd);
    motorGo(MOTOR_RIGHT, FORWARD, rightWhSpd);
  }
}


// Motor controls.
// "Monster Motor Shield. https://github.com/sparkfun/Monster_Moto_Shield/tree/Hv12Fv10
// VNH2SP30 Dual Driver module for Arduino Uno.
void motorOff(int motor)
{
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

// Trigger when button is LOW. Else, infinitely loop and lock robot.
void awaitStart() {
  while (!battleTrig) {
    if (!digitalRead(STARTBTN)) {
      battleTrig = true;
    }
  }

  delay(5000);
}

void calibrateBlack() {
  // Read average, so that we don't consider spurious data (e.g. area that's too shiny)
  // The more reflected, the lesser the value.
  int aveFL = 0; // Front left, FL
  int aveFR = 0;
  int aveBL = 0;
  int aveBR = 0; // Back right, BR

  for (int i = 0; i < IR_CALIBRATION_SAMPLES; i++) {
    int instantFL = analogRead(irAnalogPins[IRFL]);
    int instantFR = analogRead(irAnalogPins[IRFR]);
    int instantBL = analogRead(irAnalogPins[IRBL]);
    int instantBR = analogRead(irAnalogPins[IRBR]);

    aveFL += instantFL;
    aveFR += instantFR;
    aveBL += instantBL;
    aveBR += instantBR;

    if (BATTLE_DEBUG) {
      char data[100];
      sprintf(data, "Read %d: FL %d FR %d BL %d BR %d", i, instantFL, instantFR, instantBL, instantBR);
      Serial.println(data);
    }
  }

  aveFL =  aveFL / IR_CALIBRATION_SAMPLES;
  aveFR = aveFR / IR_CALIBRATION_SAMPLES;
  aveBL =  aveBL / IR_CALIBRATION_SAMPLES;
  aveBR =  aveBR / IR_CALIBRATION_SAMPLES;

  IR_ANALOG_CALIB = (int) ((aveFL + aveFR + aveBL + aveBR) / 4);

  if (BATTLE_DEBUG) {
    char data[100];
    sprintf(data, "Average (Samples = %d): aFL %d aFR %d aBL %d aBR %d, totAve %d",
            IR_CALIBRATION_SAMPLES, aveFL, aveFR, aveBL, aveBR, IR_ANALOG_CALIB);
    Serial.println(data);

    char data2[100];
    sprintf("Calibration complete, in-arena average: %d", IR_ANALOG_CALIB);
    Serial.println(data2);
  }
}
