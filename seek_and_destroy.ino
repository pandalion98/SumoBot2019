#include<NewPing.h>
#include <EEPROM.h>

#define BATTLE_DEBUG true // Enables serial outputs

// CONTROLS!!!!!!1!
#define FORWARD   1
#define BACKWARD  2
#define BRAKEGND 3
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1
#define STARTBTN 26
#define WHITECALIBBTN 27


// ---------- VNH2SP30 pin definitions ----------
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
// ---------- VNH2SP30 pin definitions ----------

// ---------- IR sensors ----------
bool whiteCalib = false;
int IR_BLACK_CALIB = 3500; // IR sensor analogRead value when placed in black area
int IR_WHITE_CALIB = 3500; // IR sensor analogRead value when placed in black area
int BORDER_DELTA = 0;

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
  pinMode(WHITECALIBBTN, INPUT_PULLUP);

  // VNH2SP30: Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }

  // VNH2SP30: Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }

  // Initialize analog IR pins
  for (int i = 0; i < 4; i++)
  {
    pinMode(irAnalogPins[i], INPUT);
  }

  // If white calibrate button is held, read and save white value to EEPROM.
  if (!digitalRead(WHITECALIBBTN)) {
    whiteCalib = true;
    calibrateBlack();
  }

  // Wait for start button press.
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
  int aveFL = 0; // Front left, fLeft
  int aveFR = 0;
  int aveBL = 0;
  int aveBR = 0; // Back right, bRight

  for (int i = 0; i < IR_SAMPLES_PER_STEP; i++) {
    aveFL += analogRead(irAnalogPins[IRFL]);
    aveFR += analogRead(irAnalogPins[IRFR]);
    aveBL += analogRead(irAnalogPins[IRBL]);
    aveBR += analogRead(irAnalogPins[IRBR]);
  }

  aveFL =  aveFL / IR_SAMPLES_PER_STEP;
  aveFR =  aveFR / IR_SAMPLES_PER_STEP;
  aveBL =  aveBL / IR_SAMPLES_PER_STEP;
  aveBR =  aveBR / IR_SAMPLES_PER_STEP;

  // We can compress this part somewhere, but we don't have time
  bool fLeftWhite = abs(aveFL -  IR_BLACK_CALIB) < BORDER_DELTA ;
  bool fRightWhite = abs(aveFR - IR_BLACK_CALIB) < BORDER_DELTA;
  bool bLeftWhite = abs(aveBL - IR_BLACK_CALIB) < BORDER_DELTA;
  bool bRightWhite = abs(aveBR - IR_BLACK_CALIB) < BORDER_DELTA;

  if (BATTLE_DEBUG) {
    char data[100];
    sprintf(data, "LiveIR FL %d FR %d BL %d BR %d \t inBorder FL:%d FR:%d BL:%d BR:%d",
            aveFL, aveFR, aveBL, aveBR, fLeftWhite, fRightWhite, bLeftWhite, bRightWhite);
    Serial.println(data);
  }

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
  if (whiteCalib) {
    // Wait for a bit to make sure the bot is stable (i.e. don't read while operator still handling bot)
    delay(5000);
  }

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

  int AVERAGE_READ = (int) ((aveFL + aveFR + aveBL + aveBR) / 4);

  // If white calibration, save white value to EEPROM.
  // Else, consider reading as black, and load white value from EEPROM.
  if (whiteCalib) {
    EEPROM.put(0, AVERAGE_READ);
    whiteCalib = false;
  } else {
    IR_BLACK_CALIB = AVERAGE_READ; // We now have a stable black value

    int IR_WHITE_CALIB;
    EEPROM.get(0, IR_WHITE_CALIB);

    // This is now the delta value between black and white areas.
    BORDER_DELTA = abs(IR_BLACK_CALIB - IR_WHITE_CALIB);
  }

  if (BATTLE_DEBUG) {
    if (whiteCalib) {
      Serial.println("WHITE CALIBRATION")
    }

    char data[100];
    sprintf(data, "Average (Samples = %d): aFL %d aFR %d aBL %d aBR %d, totAve %d",
            IR_CALIBRATION_SAMPLES, aveFL, aveFR, aveBL, aveBR, IR_BLACK_CALIB);
    Serial.println(data);

    char data2[100];
    sprintf("Calibration complete, in-arena average: %d", IR_BLACK_CALIB);
    Serial.println(data2);
  }
}
