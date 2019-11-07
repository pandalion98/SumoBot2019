#include<NewPing.h>

// CONTROLS!!!!!!1!
#define FORWARD   1
#define BACKWARD  2
#define BRAKEGND 3
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

// ---------- VNH2SP30 pin definitions ----------
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
// ---------- VNH2SP30 pin definitions ----------

// ---------- IR sensors ----------
#define IR_ANALOG_CALIB 3500 // IR sensor analogRead value when placed in black area
#define IR_SAMPLES_PER_STEP 3 // Number of samples for data averaging. See code.
#define IRFL 0 // IR, Front left
#define IRFR 1
#define IRBL 2
#define IRBR 3

int irAnalogPins[4] = {A8, A9, A11, A10};
// ---------- IR sensors ----------

bool enemyTargeted = false;
bool rescueMode = false;

void setup()
{
  Serial.begin(250000);

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

  pinMode(26, INPUT);

}

bool initialTrig = true;
void loop()
{
  if (initialTrig) {
    delay(5000);
  }
  checkSurroundings();
}

void checkSurroundings() {
  char data[100];

  // Read average, so that we don't consider spurious data (e.g. area that's too shiny)
  // The more reflected, the lesser the value.
  int averagefLeftWhite = 9999; // Front left, fLeft
  int averagefRightWhite = 9999;
  int averagebLeftWhite = 9999;
  int averagebRightWhite = 9999; // Back right, bRight

  for (int i = 0; i < IR_SAMPLES_PER_STEP; i++) {
    averagefLeftWhite += analogRead(irAnalogPins[IRFL]);
    averagefRightWhite += analogRead(irAnalogPins[IRFR]);
    averagebLeftWhite += analogRead(irAnalogPins[IRBL]);
    averagebRightWhite += analogRead(irAnalogPins[IRBR]);
  }

  averagefLeftWhite = (int) (averagefLeftWhite / IR_SAMPLES_PER_STEP);
  averagefRightWhite = (int) (averagefRightWhite / IR_SAMPLES_PER_STEP);
  averagebLeftWhite = (int) (averagebLeftWhite / IR_SAMPLES_PER_STEP);
  averagebRightWhite = (int) (averagebRightWhite / IR_SAMPLES_PER_STEP);

  sprintf(data, "FL %d FR %d BL %d BR %d", averagefLeftWhite, averagefRightWhite, averagebLeftWhite, averagebRightWhite);
  Serial.println(data);

  // We can compress this part somewhere, but we don't have time
  bool fLeftWhite = averagefLeftWhite < IR_ANALOG_CALIB;
  bool fRightWhite = averagefRightWhite < IR_ANALOG_CALIB;
  bool bLeftWhite = averagebLeftWhite < IR_ANALOG_CALIB;
  bool bRightWhite = averagebRightWhite < IR_ANALOG_CALIB;

  int leftWhSpd = random(127, 255);
  int rightWhSpd = random(127, 255);

  // Decision tree. What to do if IR detects we're in white.
  if (fLeftWhite || fRightWhite) {         // Whole FRONT face in white
    rescueMode = true;
    motorGo(MOTOR_LEFT, BACKWARD, leftWhSpd);
    motorGo(MOTOR_RIGHT, BACKWARD, rightWhSpd);
    // Full reverse for X ms
  } else if (bLeftWhite || bRightWhite || initialTrig) {
    initialTrig = false;
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
