//This is taken from https://www.reddit.com/r/Dobot/comments/45ilan/controlling_dobot_with_ramps_14

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include <QueueArray.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "U8glib.h"

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define Q_STEP_PIN         36
#define Q_DIR_PIN          34
#define Q_ENABLE_PIN       30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define FAN_PIN            9

#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       10
#define HEATER_1_PIN       8
#define TEMP_0_PIN         13
#define TEMP_1_PIN         14
#define zPlus              40
#define zMinus             42

static const int  BACK_ARM_LENGTH  =  280;
static const int  FORE_ARM_LENGTH  =  240;

static const long MICRO_STEPS_PER_ROUND = 6400;
static const long X_MOTOR_STEPS = 89600;
static const long Y_MOTOR_STEPS = 121600;
static const double Z_MOTOR_STEPS = 48000;

static const double X_INIT_ANGLE = 24.1;
static const double Y_INIT_ANGLE = 13.05 ;

AccelStepper XAxis(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper YAxis(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper ZAxis(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// Stepper Travel Variables
long TravelX;  // Used to store the X value entered in the Serial Monitor
int move_finished = 1; // Used to check if move is completed
long initial_homing = -1; // Used to Home Stepper at startup
long initial_y_homing = -1; // Used to Home Stepper at startup
long initial_z_homing = -1; // Used to Home Stepper at startup
long initial_x_homing = -1; // Used to Home Stepper at startup

long direction = 10000;
long distanceToGo = 0;

int numberOfPlates = 3;
int destinationTower = 1;
int intermediateTower = 2;

int** movesArray = NULL;
int numberOfMoves = 0;
int currentMove = -1;
int currentTowerStatus[] = {0, 0, 0};
int currentTower = 0;
int currentInstructionType = 0;

boolean isHanoiInputReady = false;
boolean isTowerApositionReady = false;
boolean isPlatesReady = false;
boolean isReadyForNextMove = true;
boolean isCurrentInstructionComplete = true;

int plateHieght = 15;
int plateFreeMoveGap = 5;
int baseX = 409;
int baseY = -216;
double baseZAngle = 23;
long baseZSteps = 0;

int isMagnetOn = 0;

long currentYsteps = 0;
long currentXsteps = 0;

long nextYsteps = 0;
long nextXsteps = 0;

static const int INST_TYPE_UP_AND_DOWN = 1;
static const int INST_TYPE_UP_ROTATE = 2;
static const int INST_TYPE_UP_MAGNET = 3;

typedef struct {
  int      type;
  long     x;
  long     y;
  long     z;
  boolean  magnetStatus;
} Instruction;

QueueArray <Instruction> queue;

// ===============  START GRAPHIC =================

U8GLIB_ST7920_128X64_1X u8g(23, 17, 16);
/* Rotary encoder (dial) pins */
#define ROT_EN_A 31
#define ROT_EN_B 33
/* Rotary encoder button pin */
#define BUTTON_DIO 35
/* Reset button pin */
#define RESET_DIO 41
/* Buzzer pin */
#define BUZZER_DIO 37

#define INIT_SCREEN_ITEMS 4
#define MAIN_MENU_ITEMS 3
#define PLATE_MENU_ITEMS 6
#define TOWER_MENU_ITEMS 2

const char *initScreenItems[INIT_SCREEN_ITEMS] = { "AKISAR", "Moving", "to", "Home Position"};

const char *mainMenuItems[MAIN_MENU_ITEMS] = { "Number of Plates : ", "Destination Tower: ", "START"};
char *mainMenuValues[MAIN_MENU_ITEMS] = { "6", "B"};

const char *plateMenuItems[PLATE_MENU_ITEMS] = { "Number of plates"};
const char *plateMenuValues[PLATE_MENU_ITEMS] = { "1", "2", "3", "4", "5", "6"};

const char *towerMenuItems[TOWER_MENU_ITEMS] = { "Destination Tower"};
const char *towerMenuValues[TOWER_MENU_ITEMS] = { "B", "C"};

uint8_t currentNumberOfMenuItems = MAIN_MENU_ITEMS;
uint8_t currentMenuItem = 0;
uint8_t currentPageNumber = 0;

uint8_t menu_redraw_required = 1;

volatile int myInterruptVar = 0;
volatile int rotary_button_checked = 0;
volatile int reset_button_checked = 0;

volatile uint8_t rotary_button_pressd = 0;
volatile uint8_t reset_button_pressd = 0;

volatile uint8_t rotary_button_check = 1;
volatile uint8_t reset_button_check = 1;

volatile byte DialCount = 120;
volatile byte PreDialCount = 120;
volatile byte DialPos = 0;
volatile byte Last_DialPos = 0;

String message1 = "Press 'START' to";
String message2 = "to go Tower A";
// ===============  END GRAPHIC =================

void setup() {

  cli();          // disable global interrupts
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // same for TCCR3B

  // set compare match register to desired timer count:  @~744 Hz
  OCR3A = 150;
  // turn on CTC mode:
  TCCR3B |= (1 << WGM32);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR3B |= (1 << CS30) | (1 << CS32);
  // enable timer compare interrupt:
  TIMSK3 |= (1 << OCIE3B);
  // enable global interrupts:
  sei();

  drawDisplay();

  baseZSteps = stepsZ(baseZAngle);

  pinMode(BUZZER_DIO, OUTPUT);
  pinMode(BUTTON_DIO, INPUT);
  digitalWrite(BUTTON_DIO, HIGH);
  pinMode(RESET_DIO, INPUT);
  digitalWrite(RESET_DIO, HIGH);
  pinMode(ROT_EN_A, INPUT);
  pinMode(ROT_EN_B, INPUT);
  digitalWrite(ROT_EN_A, HIGH);
  digitalWrite(ROT_EN_B, HIGH);

  pinMode(FAN_PIN , OUTPUT);
  pinMode(HEATER_0_PIN , OUTPUT);
  pinMode(HEATER_1_PIN , OUTPUT);
  pinMode(LED_PIN  , OUTPUT);

  pinMode(X_STEP_PIN  , OUTPUT);
  pinMode(X_DIR_PIN    , OUTPUT);
  pinMode(X_ENABLE_PIN    , OUTPUT);
  pinMode(X_MIN_PIN, INPUT);
  pinMode(X_MAX_PIN, INPUT);

  pinMode(Y_STEP_PIN  , OUTPUT);
  pinMode(Y_DIR_PIN    , OUTPUT);
  pinMode(Y_ENABLE_PIN    , OUTPUT);
  pinMode(Y_MIN_PIN, INPUT);
  pinMode(Y_MAX_PIN, INPUT);

  pinMode(Z_STEP_PIN  , OUTPUT);
  pinMode(Z_DIR_PIN    , OUTPUT);
  pinMode(Z_ENABLE_PIN    , OUTPUT);
  pinMode(Z_MIN_PIN, INPUT);
  pinMode(Z_MAX_PIN, INPUT);

  pinMode(E_STEP_PIN  , OUTPUT);
  pinMode(E_DIR_PIN    , OUTPUT);
  pinMode(E_ENABLE_PIN    , OUTPUT);

  pinMode(Q_STEP_PIN  , OUTPUT);
  pinMode(Q_DIR_PIN    , OUTPUT);
  pinMode(Q_ENABLE_PIN    , OUTPUT);

  pinMode(zPlus, INPUT_PULLUP);
  pinMode(zMinus, INPUT_PULLUP);

  digitalWrite(X_ENABLE_PIN    , LOW);
  digitalWrite(Y_ENABLE_PIN    , LOW);
  digitalWrite(Z_ENABLE_PIN    , LOW);
  digitalWrite(E_ENABLE_PIN    , LOW);
  digitalWrite(Q_ENABLE_PIN    , LOW);

  digitalWrite(HEATER_0_PIN    , LOW);

  XAxis.setMaxSpeed(500);
  XAxis.setAcceleration(100);

  YAxis.setMaxSpeed(500);
  //YAxis.setSpeed(100);
  YAxis.setAcceleration(100);

  ZAxis.setMaxSpeed(500);
  //ZAxis.setSpeed(100);
  ZAxis.setAcceleration(100);

  Serial.begin(9600);

  homeYAxis();
  homeXAxis();
  homeZAxis();

  //gotoInitYPosition();
  //homeXAxis();
  gotoInitZPosition();

  setMotorSpeedAndPosition();

  currentPageNumber = 1;
  currentMenuItem = 0;
  //hanoiNoOfMoves();
  //goToStartPoint();
}

void setMotorSpeedAndPosition(void) {
  XAxis.setCurrentPosition(0);
  YAxis.setCurrentPosition(0);
  ZAxis.setCurrentPosition(0);

  XAxis.setMaxSpeed(10000.0);
  YAxis.setMaxSpeed(20000.0);
  ZAxis.setMaxSpeed(5000.0);

  XAxis.setAcceleration(1000.0);
  YAxis.setAcceleration(1000.0);
  ZAxis.setAcceleration(500.0);
}

void initialize(void) {
  Serial.println("=======================================================");
  Serial.println("=                                                     =");
  Serial.println("=                                                     =");
  Serial.println("=             START FROM THE BEGINING                 =");
  Serial.println("=                                                     =");
  Serial.println("=                                                     =");
  Serial.println("=======================================================");
}

void homeZAxis(void) {
  Serial.println("=======================================================");
  Serial.println("=                    Z Axis homing                    =");
  Serial.println("=======================================================");

  ZAxis.setMaxSpeed(5000);
  //ZAxis.setSpeed(100);
  ZAxis.setAcceleration(500);
  initial_z_homing = 0;

  while (digitalRead(Z_MAX_PIN)) {  // Make the Stepper move CCW until the switch is activated
    ZAxis.moveTo(initial_z_homing);  // Set the position to move to
    initial_z_homing++;  // Decrease by 1 for next move if needed
    ZAxis.run();  // Start moving the stepper
    //delay(5);
  }

  Serial.println("Max steps :- ");
  Serial.println(initial_z_homing);

  //delay(1);

  ZAxis.setCurrentPosition(0);  // Set the current position as zero for now
  ZAxis.setMaxSpeed(100);
  //ZAxis.setSpeed(100);
  ZAxis.setAcceleration(100);
  initial_z_homing = 0;


  while (!digitalRead(Z_MAX_PIN)) { // Make the Stepper move CW until the switch is deactivated
    ZAxis.moveTo(initial_z_homing);
    ZAxis.run();
    initial_z_homing--;
    //delay(5);
  }

  Serial.println("Max steps :- ");
  Serial.println(initial_z_homing);

  ZAxis.setCurrentPosition(0);
}

void homeYAxis(void) {
  Serial.println("=======================================================");
  Serial.println("=                    Y Axis homing                    =");
  Serial.println("=======================================================");

  YAxis.setMaxSpeed(10000);
  //YAxis.setSpeed(100);
  YAxis.setAcceleration(2000);
  initial_homing = -1;

  while (digitalRead(Y_MIN_PIN)) {  // Make the Stepper move CCW until the switch is activated
    if (!digitalRead(Y_MAX_PIN)) {
      adjustXAxis();
    }
    YAxis.moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    YAxis.run();  // Start moving the stepper
    //delay(5);
  }

  Serial.println("Max steps :- ");
  Serial.println(initial_homing);

  //delay(1);

  YAxis.setCurrentPosition(0);  // Set the current position as zero for now
  YAxis.setMaxSpeed(100);
  //YAxis.setSpeed(100);
  YAxis.setAcceleration(100);
  initial_homing = 1;


  while (!digitalRead(Y_MIN_PIN)) { // Make the Stepper move CW until the switch is deactivated
    YAxis.moveTo(initial_homing);
    YAxis.run();
    initial_homing++;
    //delay(5);
  }

  Serial.println("Max steps :- ");
  Serial.println(initial_homing);
}

void  adjustXAxis(void) {
  Serial.println("=======================================================");
  Serial.println("=                X Axis adjust homing                 =");
  Serial.println("=======================================================");

  XAxis.setMaxSpeed(1000);
  //XAxis.setSpeed(100);
  XAxis.setAcceleration(100);
  initial_x_homing = 0;

  while (initial_x_homing < 60000) {  // Make the X Stepper move CW until the switch is activated
    XAxis.moveTo(initial_x_homing);  // Set the position to move to
    initial_x_homing++;  // Decrease by 1 for next move if needed
    XAxis.run();  // Start moving the stepper
    //delay(5);
  }

  Serial.println("Max steps :- ");
  Serial.println(initial_x_homing);

  //delay(1);

  XAxis.setCurrentPosition(0);  // Set the current position as zero for now
}

void homeXAxis(void) {
  Serial.println("=======================================================");
  Serial.println("=                    X Axis homing                    =");
  Serial.println("=======================================================");

  XAxis.setMaxSpeed(1000);
  //XAxis.setSpeed(100);
  XAxis.setAcceleration(100);
  initial_x_homing = 0;

  while (digitalRead(Y_MAX_PIN) && digitalRead(X_MIN_PIN)) {  // Make the Stepper move CCW until the switch is activated
    XAxis.moveTo(initial_x_homing);  // Set the position to move to
    initial_x_homing--;  // Decrease by 1 for next move if needed
    XAxis.run();  // Start moving the stepper
    //delay(5);
  }

  Serial.println("Max steps :- ");
  Serial.println(initial_x_homing );

  // delay(1);

  XAxis.setCurrentPosition(0);  // Set the current position as zero for now
  XAxis.setMaxSpeed(1000);
  //XAxis.setSpeed(100);
  XAxis.setAcceleration(100);
  initial_x_homing = 1;


  while (!digitalRead(Y_MAX_PIN)) { // Make the Stepper move CW until the switch is deactivated
    XAxis.moveTo(initial_x_homing);
    XAxis.run();
    initial_x_homing++;
    //delay(5);
  }

  while (!digitalRead(X_MIN_PIN)) { // Make the Stepper move CW until the switch is deactivated
    XAxis.moveTo(initial_x_homing);
    XAxis.run();
    initial_x_homing++;
    //delay(5);
  }

  XAxis.setCurrentPosition(0);
  initial_x_homing = 1;

  while (initial_x_homing < 200) {
    XAxis.moveTo(initial_x_homing);
    XAxis.run();
    initial_x_homing++;
  }

  Serial.println("Max steps :- ");
  Serial.println(initial_x_homing);

  XAxis.setCurrentPosition(0);
}

void gotoInitYPosition(void) {
  YAxis.setCurrentPosition(0);
  YAxis.setMaxSpeed(20000.0);
  YAxis.setAcceleration(2000.0);
  initial_y_homing = 0;


  while (initial_y_homing < 15639) {
    YAxis.moveTo(initial_y_homing);
    YAxis.run();
    initial_y_homing++;
  }

  YAxis.setCurrentPosition(0);
}

void gotoInitZPosition(void) {
  ZAxis.setCurrentPosition(0);  // Set the current position as zero for now
  ZAxis.setMaxSpeed(5000);
  //ZAxis.setSpeed(500);
  ZAxis.setAcceleration(500);
  //  initial_z_homing = 0;
  //
  //
  //  while (initial_z_homing > -26200) { // Make the Stepper move CW until the switch is deactivated
  //    ZAxis.moveTo(initial_z_homing);
  //    ZAxis.run();
  //    initial_z_homing--;
  //    //delay(5);
  //  }
  //
  //  Serial.println("Max steps :- ");
  //  Serial.println(initial_z_homing);
  //
  //  ZAxis.setCurrentPosition(0);

  ZAxis.moveTo(-20000);

  while (ZAxis.distanceToGo() != 0) {
    ZAxis.run();
  }

  ZAxis.setCurrentPosition(0);
}

void goToStartPoint(void) {
  ZAxis.moveTo(-(baseZSteps));

  while (ZAxis.distanceToGo() != 0) {
    ZAxis.run();
  }

  ZAxis.setCurrentPosition(0);

  ZAxis.setMaxSpeed(10000);
  ZAxis.setAcceleration(500.0);

  double initY = baseY + (numberOfPlates * plateHieght) + plateFreeMoveGap;
  //  double initX = baseX / cos(rad(baseZAngle));
  double initX = baseX;

  stepsXandY(initX, initY);

  YAxis.moveTo(nextYsteps);
  XAxis.moveTo(nextXsteps);

  while (YAxis.distanceToGo() != 0 || XAxis.distanceToGo() != 0) {
    YAxis.run();
    XAxis.run();
  }

  XAxis.setCurrentPosition(0);
  YAxis.setCurrentPosition(0);

  currentMove = -1;
  currentTowerStatus[0] = numberOfPlates;
}

void goToTowerBFromTowerC(void) {
  ZAxis.moveTo(-(baseZSteps));

  while (ZAxis.distanceToGo() != 0) {
    ZAxis.run();
  }

  ZAxis.setCurrentPosition(0);
}

long getDeltaXsteps() {
  return nextXsteps - currentXsteps;
}

long getDeltaYsteps() {
  return nextYsteps - currentYsteps;
}

void addUpAndDownInstruction(double nextTowerX, double nextTowerY) {
  stepsXandY(nextTowerX, nextTowerY);

  Instruction instruction = { INST_TYPE_UP_AND_DOWN, getDeltaXsteps(), getDeltaYsteps(), 0, false };
  queue.enqueue (instruction);
}

void addLeftRightInstruction(long zSteps) {
  Instruction instruction = { INST_TYPE_UP_ROTATE, 0, 0, zSteps, false };
  queue.enqueue (instruction);
}

void addMagnetInstruction(boolean magnetStatus) {
  Instruction instruction = { INST_TYPE_UP_MAGNET, 0, 0, 0, magnetStatus };
  queue.enqueue (instruction);
}


void subMoveStepCommon(int fromTower, int toTower, int carryPlate) {
  int towerDiff = toTower - fromTower;

  if (towerDiff < 0) {
    if (towerDiff == -1) {
      double highestTowerY =  baseY + plateFreeMoveGap;

      if (currentTowerStatus[fromTower - 1] > currentTowerStatus[fromTower]) {
        highestTowerY = highestTowerY + ((currentTowerStatus[fromTower - 1] + carryPlate) * plateHieght);
      } else {
        highestTowerY = highestTowerY + ((currentTowerStatus[fromTower] + carryPlate) * plateHieght);
      }

      //      double toTowerX = (toTower == 1) ? baseX : (baseX / cos(rad(baseZAngle)));
      double toTowerX = baseX;
      addUpAndDownInstruction(toTowerX, highestTowerY);

      long zSteps = -(baseZSteps);
      Serial.println("[MOTOR] : Z-STEPS: "); Serial.print(zSteps);
      addLeftRightInstruction(zSteps);
      // up plate hieght
      // move to right (-)  * 23
    } else {
      double highestTowerY =  baseY + plateFreeMoveGap;

      if ((currentTowerStatus[fromTower - 1] > currentTowerStatus[fromTower]) && (currentTowerStatus[fromTower - 1] > currentTowerStatus[fromTower - 2])) {
        highestTowerY = highestTowerY + ((currentTowerStatus[fromTower - 1] + carryPlate) * plateHieght);
      } else if ((currentTowerStatus[fromTower - 2] > currentTowerStatus[fromTower]) && (currentTowerStatus[fromTower - 2] > currentTowerStatus[fromTower - 1])) {
        highestTowerY = highestTowerY + ((currentTowerStatus[fromTower - 2] + carryPlate) * plateHieght);
      } else {
        highestTowerY = highestTowerY + ((currentTowerStatus[fromTower] + carryPlate) * plateHieght);
      }

      //      double toTowerX = (toTower == 1) ? baseX : (baseX / cos(rad(baseZAngle)));
      double toTowerX = baseX;
      addUpAndDownInstruction(toTowerX, highestTowerY);

      long zSteps = -(baseZSteps * 2);
      Serial.println("[MOTOR] : Z-STEPS: "); Serial.print(zSteps);
      addLeftRightInstruction(zSteps);
      // up plate hieght
      // move to right (-)  * 23 * 2
    }
  } else if (towerDiff > 0) {
    if (towerDiff == 1) {
      double highestTowerY =  baseY + plateFreeMoveGap;

      if (currentTowerStatus[fromTower + 1] > currentTowerStatus[fromTower]) {
        highestTowerY = highestTowerY + ((currentTowerStatus[fromTower + 1] + carryPlate) * plateHieght);
      } else {
        highestTowerY = highestTowerY + ((currentTowerStatus[fromTower] + carryPlate) * plateHieght);
      }

      //      double toTowerX = (toTower == 1) ? baseX : (baseX / cos(rad(baseZAngle)));
      double toTowerX = baseX;
      addUpAndDownInstruction(toTowerX, highestTowerY);

      long zSteps = baseZSteps;
      Serial.println("[MOTOR] : Z-STEPS: "); Serial.print(zSteps);
      addLeftRightInstruction(zSteps);
      // up plate hieght
      // move to left  * 23
    } else {
      double highestTowerY =  baseY + plateFreeMoveGap;

      if ((currentTowerStatus[fromTower + 1] > currentTowerStatus[fromTower]) && (currentTowerStatus[fromTower + 1] > currentTowerStatus[fromTower + 2])) {
        highestTowerY = highestTowerY + ((currentTowerStatus[fromTower + 1] + carryPlate) * plateHieght);
      } else if ((currentTowerStatus[fromTower + 2] > currentTowerStatus[fromTower]) && (currentTowerStatus[fromTower + 2] > currentTowerStatus[fromTower + 1])) {
        highestTowerY = highestTowerY + ((currentTowerStatus[fromTower + 2] + carryPlate) * plateHieght);
      } else {
        highestTowerY = highestTowerY + ((currentTowerStatus[fromTower] + carryPlate) * plateHieght);
      }

      //      double toTowerX = (toTower == 1) ? baseX : (baseX / cos(rad(baseZAngle)));
      double toTowerX = baseX;
      addUpAndDownInstruction(toTowerX, highestTowerY);

      long zSteps = baseZSteps * 2;
      Serial.println("[MOTOR] : Z-STEPS: "); Serial.print(zSteps);
      addLeftRightInstruction(zSteps);
      // up plate hieght
      // move to left  * 23 * 2
    }
  }
}

void subMoveStep_2(int fromTower, int toTower) {
  double fromTowerX = baseX;
  double fromTowerY = baseY + (currentTowerStatus[fromTower] * plateHieght);

  addUpAndDownInstruction(fromTowerX, fromTowerY);
  addMagnetInstruction(true);

  subMoveStepCommon( fromTower,  toTower, 1);

  fromTowerX = baseX;
  fromTowerY = baseY + ((currentTowerStatus[toTower] + 1) * plateHieght);
  addUpAndDownInstruction(fromTowerX, fromTowerY);
  addMagnetInstruction(false);
}


void loop () {
  drawDisplay();

  if (!isTowerApositionReady && isHanoiInputReady) {
    hanoiNoOfMoves();
    goToStartPoint();
    isTowerApositionReady = true;
    message1 = "Place the plates and";
    message2 = "press 'START' again";
    menu_redraw_required = 1;
  }

  //      Serial.println("numberOfPlates >>>> ");
  //      Serial.print(numberOfPlates);
  //      Serial.print(" destinationTower >>>> ");
  //      Serial.print(destinationTower);
  //      Serial.print(" intermediateTower >>>> ");
  //      Serial.print(intermediateTower);

  if (isPlatesReady) {
    if (isReadyForNextMove) {
      currentMove++;
      if (currentMove < numberOfMoves) {

        Serial.println(" ");
        Serial.print("[MOTOR] : Current Move: "); Serial.print(currentMove);

        int fromTower = movesArray[currentMove][1];
        int toTower = movesArray[currentMove][2];

        Serial.println(" ");
        Serial.print("[MOTOR] : currentTower >>>> : "); Serial.print(currentTower); Serial.print(" fromTower: "); Serial.print(fromTower); Serial.print(" toTower: "); Serial.print(toTower);
        Serial.print("[MOTOR] : Current Move: "); Serial.print(currentMove);


        //      if (currentMove > 0) {
        //        currentTower = movesArray[currentMove - 1][2];
        //      }

        if (currentTower == fromTower) {
          Serial.println(" ");
          Serial.print("[MOTOR] : currentTower 2: "); Serial.print(currentTower); Serial.print(" fromTower: "); Serial.print(fromTower); Serial.print(" toTower: "); Serial.print(toTower);
          subMoveStep_2(fromTower, toTower);
          currentTower = toTower;
        } else {
          Serial.println(" ");
          Serial.print("[MOTOR] : currentTower 3: "); Serial.print(currentTower); Serial.print(" fromTower: "); Serial.print(fromTower); Serial.print(" toTower: "); Serial.print(toTower);
          subMoveStepCommon(currentTower, fromTower, 0);
          currentTower = fromTower;

          Serial.println(" ");
          Serial.print("[MOTOR] : currentTower 4: "); Serial.print(currentTower); Serial.print(" fromTower: "); Serial.print(fromTower); Serial.print(" toTower: "); Serial.print(toTower);
          subMoveStep_2(fromTower, toTower);
          currentTower = toTower;
        }

        currentTowerStatus[fromTower]--;
        currentTowerStatus[toTower]++;

      } else {
        Serial.println(" <<<<<<<<<<<<<<< THIS IS THE LAST MOVE >>>>>>>>>>>>>>");
      }

      isReadyForNextMove = false;
    }

    Instruction instruction;

    if (!queue.isEmpty ()) {
      if (isCurrentInstructionComplete) {
        instruction = queue.dequeue ();

        if (instruction.type == INST_TYPE_UP_AND_DOWN) {
          Serial.println(" ");
          Serial.print("[MOTOR] : Current Instruction : UP_AND_DOWN"); Serial.print(" X: "); Serial.print(instruction.x); Serial.print(" Y: "); Serial.print(instruction.y);
          currentInstructionType = INST_TYPE_UP_AND_DOWN;
          YAxis.moveTo(instruction.y);
          XAxis.moveTo(instruction.x);
          isCurrentInstructionComplete = false;
        } else if (instruction.type == INST_TYPE_UP_ROTATE) {
          Serial.println(" ");
          Serial.print("[MOTOR] : Current Instruction : ROTATE"); Serial.print(" Z: "); Serial.print(instruction.z);
          currentInstructionType = INST_TYPE_UP_ROTATE;
          ZAxis.moveTo(instruction.z);
          isCurrentInstructionComplete = false;
        } else if (instruction.type == INST_TYPE_UP_MAGNET) {
          Serial.println(" ");
          Serial.print("[MOTOR] : Current Instruction : MAGNET "); Serial.print(instruction.magnetStatus);
          currentInstructionType = INST_TYPE_UP_MAGNET;
          if (instruction.magnetStatus) {
            digitalWrite(HEATER_0_PIN, HIGH);

          } else {
            digitalWrite(HEATER_0_PIN    , LOW);
          }
          isCurrentInstructionComplete = true;
        }
        Serial.println(" ");
        Serial.print("INSTRUCTION TYPE "); Serial.print(currentInstructionType);
      }
    } else if (currentMove < numberOfMoves) {
      isReadyForNextMove = true;
    } else if (isCurrentInstructionComplete && (currentMove == numberOfMoves)) {
      Serial.println(" <<<<<<<<<<<<<<< All done >>>>>>>>>>>>>>");

      homeYAxis();
      homeXAxis();

      if (currentTower == 2) {
        Serial.println(">>>>>>>>>>>>>>>>>> CURRENT TOWER IS C >>>>>>>>>>>>>>>>>>>>>>>>");
        goToTowerBFromTowerC();
      }

      isHanoiInputReady = false;
      isTowerApositionReady = false;
      isPlatesReady = false;

      numberOfMoves = 0;
      currentMove = -1;
      currentTowerStatus[0] = 0;
      currentTowerStatus[1] = 0;
      currentTowerStatus[2] = 0;
      currentTower = 0;
      currentInstructionType = 0;

      isReadyForNextMove = true;
      isCurrentInstructionComplete = true;

      setMotorSpeedAndPosition();

      message1 = "Press 'START' to";
      message2 = "to go Tower A";
      menu_redraw_required = 1;
    }

    if (YAxis.distanceToGo() != 0) {
      //Serial.println(">>>>>>>>>>>>>>>>>> 3");
      YAxis.run();
    }

    if (XAxis.distanceToGo() != 0) {
      //Serial.println(">>>>>>>>>>>>>>>>>> 4");
      XAxis.run();
    }

    if (YAxis.distanceToGo() == 0 && XAxis.distanceToGo() == 0 && currentInstructionType == INST_TYPE_UP_AND_DOWN) {
      Serial.println(">>>>>>>>>>>>>>>>>> 5");
      isCurrentInstructionComplete = true;
      XAxis.setCurrentPosition(0);
      YAxis.setCurrentPosition(0);
      //      delay(3000);
    }

    if (ZAxis.distanceToGo() != 0) {
      //Serial.println(">>>>>>>>>>>>>>>>>> 6");
      ZAxis.run();
    }

    //    if(currentInstructionType == INST_TYPE_UP_ROTATE){
    //      Serial.println(">>>>>>>>>>>>>>>>>> 8");
    //      if(ZAxis.distanceToGo() == 0){
    //        Serial.println(">>>>>>>>>>>>>>>>>> 9");
    //      }
    //    }

    //    Serial.print("INSTRUCTION TYPE "); Serial.print(instruction.type);

    if (ZAxis.distanceToGo() == 0 && currentInstructionType == INST_TYPE_UP_ROTATE) {
      Serial.println(">>>>>>>>>>>>>>>>>> 7");
      isCurrentInstructionComplete = true;
      ZAxis.setCurrentPosition(0);
      //      delay(3000);
    }


    // add robot movements here with above queue

  } else {
    //    Serial.println("FINISHED!");
    // todo call init procedure
  }

  // add input read for display as well
}


void hanoiNoOfMoves(void) {
  numberOfMoves = (round(pow(2, numberOfPlates))) - 1;

  //  Serial.println("Hanoi plates :- ");
  //  Serial.print(numberOfPlates);
  //  Serial.print(" Number Of Moves :- ");
  //  Serial.print(numberOfMoves);
  //  Serial.print(" Pow function :- ");
  //  Serial.print(pow(2, numberOfPlates));

  if (movesArray != NULL) {
    movesArray = (int**) realloc(movesArray, numberOfMoves * sizeof(int));
  } else {
    movesArray = (int**) malloc(numberOfMoves * sizeof(int));
  }

  for (int iRow = 0 ; iRow < numberOfMoves ; iRow++)
  {
    movesArray[iRow] = (int *) malloc(3 * sizeof(int));
  }

  //  Serial.println("Hanoi plates :- ");
  //  Serial.print(numberOfPlates);
  //  Serial.print(" Number Of Moves :- ");
  //  Serial.print(numberOfMoves);
  //  Serial.print(" Pow function :- ");
  //  Serial.print(pow(2, numberOfPlates));


  currentMove = -1;
  towerOfHanoi(numberOfPlates, 0, destinationTower, intermediateTower);

  for (int iRow = 0 ; iRow < numberOfMoves ; iRow++)
  {
    Serial.println(" ");
    Serial.print( movesArray[iRow][0]); Serial.print( movesArray[iRow][1]); Serial.print( movesArray[iRow][2]);
  }
}

void towerOfHanoi(int diskNumber, int from_rod, int to_rod, int aux_rod)
{
  Serial.println(" ");
  if (diskNumber == 1)
  {
    currentMove ++;
    movesArray[currentMove][0] = 1;
    movesArray[currentMove][1] = from_rod;
    movesArray[currentMove][2] = to_rod;
    Serial.print("Move disk 1 from rod "); Serial.print(from_rod); Serial.print(" to rod "); Serial.print(to_rod);
    return;
  }
  towerOfHanoi(diskNumber - 1, from_rod, aux_rod, to_rod);

  currentMove ++;
  movesArray[currentMove][0] = diskNumber;
  movesArray[currentMove][1] = from_rod;
  movesArray[currentMove][2] = to_rod;
  Serial.println(" ");
  Serial.print("Move disk "); Serial.print(diskNumber); Serial.print(" from rod "); Serial.print(from_rod); Serial.print(" to rod "); Serial.print(to_rod);
  towerOfHanoi(diskNumber - 1, aux_rod, to_rod, from_rod);
}

double lawOfCosines(double a, double b, double c) {
  return acos((a * a + b * b - c * c) / (2 * a * b));
}

double distance(double x, double y) {
  return sqrt(x * x + y * y);
}

double deg(double rad) {
  return rad * 180 / PI;
}

double rad(double deg) {
  return deg *  PI / 180;
}

void stepsXandY(double x, double y) {
  double dist = distance(x, y);
  double D1 = atan2(y, x);

  double D2 = lawOfCosines(dist, BACK_ARM_LENGTH, FORE_ARM_LENGTH);

  double A1 = D1 + D2;

  double A2 = lawOfCosines(BACK_ARM_LENGTH, FORE_ARM_LENGTH, dist);

  double backArmAngle = (90 - deg(A1));
  double foreArmAngle = (deg(A2) - backArmAngle);

  double backArmMoveAngle = backArmAngle - Y_INIT_ANGLE;
  double foreArmMoveAngle = foreArmAngle - X_INIT_ANGLE;

  double backArmSteps = Y_MOTOR_STEPS / 360 * backArmMoveAngle;
  double foreArmSteps = X_MOTOR_STEPS / 360 * foreArmMoveAngle;

  currentYsteps = nextYsteps;
  nextYsteps = round (backArmSteps);

  currentXsteps = nextXsteps;
  nextXsteps = round (foreArmSteps);

  //  Serial.println(" ");
  //  Serial.print("A1 rad : "); Serial.print(A1); Serial.print("  A1 deg : "); Serial.print(backArmAngle); Serial.print("  Y move angle : "); Serial.print(backArmMoveAngle);
  //  Serial.print("  Y steps : "); Serial.print(backArmSteps);
  //  Serial.println(" ");
  //  Serial.print("A2 rad : "); Serial.print(A2); Serial.print("  A2 deg : "); Serial.print(foreArmAngle); Serial.print("  X move angle : "); Serial.print(foreArmMoveAngle);
  //  Serial.print("  X steps : "); Serial.print(foreArmSteps);
}

long stepsZ(double zAxixAngle) {
  long zSteps = round(Z_MOTOR_STEPS / 360 * zAxixAngle);
  //  Serial.println(" ");
  //  Serial.print("Z steps : "); Serial.print(zSteps);
  return zSteps;
}



void drawCurrentPage(void) {
  if (0 == currentPageNumber) {
    drawInitScreen();
  } else if (1 == currentPageNumber) {
    drawFirstPage();
  } else if (2 == currentPageNumber) {
    drawNumberOfPlatesPage();
  } else if (3 == currentPageNumber) {
    drawDestinationTowerPage();
  }
}

void drawInitScreen(void) {
  uint8_t i, h;
  u8g_uint_t w, d, hieght;

  u8g.setFont(u8g_font_fub20);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();
  u8g.setDefaultForegroundColor();

  h = u8g.getFontAscent() - u8g.getFontDescent();
  w = u8g.getWidth();
  //hieght = u8g.getHeight();
  d = 5;

  d = (w - u8g.getStrWidth(initScreenItems[0])) / 2;
  u8g.drawStr(d, 2, initScreenItems[0]);

  u8g.setFont(u8g_font_6x12);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();

  //  w = u8g.getWidth();
  //hieght = u8g.getHeight();

  d = (w - u8g.getStrWidth(initScreenItems[1])) / 2;
  u8g.drawStr(d, (h + 5), initScreenItems[1]);

  d = (w - u8g.getStrWidth(initScreenItems[2])) / 2;
  u8g.drawStr(d, (h + 15), initScreenItems[2]);

  d = (w - u8g.getStrWidth(initScreenItems[3])) / 2;
  u8g.drawStr(d, (h + 25), initScreenItems[3]);
}

void drawNumberOfPlatesPage(void) {
  uint8_t i, h;
  u8g_uint_t w, d, hieght;

  u8g.setFont(u8g_font_6x12);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();
  u8g.setDefaultForegroundColor();

  h = u8g.getFontAscent() - u8g.getFontDescent();
  w = u8g.getWidth();
  //hieght = u8g.getHeight();
  d = 5;

  d = (w - u8g.getStrWidth(plateMenuItems[0])) / 2;
  u8g.drawStr(d, 2, plateMenuItems[0]);

  u8g.setFont(u8g_font_osb35);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();

  w = u8g.getWidth();
  //hieght = u8g.getHeight();

  d = (w - u8g.getStrWidth(plateMenuValues[currentMenuItem])) / 2;
  u8g.drawStr(d, (h + 10), plateMenuValues[currentMenuItem]);
  //Serial.print("H >> "); Serial.println(h); Serial.print("W >> "); Serial.println(w); Serial.print("hieght >> "); Serial.println(hieght);
}

void drawDestinationTowerPage(void) {
  uint8_t i, h;
  u8g_uint_t w, d, hieght;

  u8g.setFont(u8g_font_6x12);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();
  u8g.setDefaultForegroundColor();

  h = u8g.getFontAscent() - u8g.getFontDescent();
  w = u8g.getWidth();
  //hieght = u8g.getHeight();

  d = (w - u8g.getStrWidth(towerMenuItems[0])) / 2;
  u8g.drawStr(d, 2, towerMenuItems[0]);

  u8g.setFont(u8g_font_osb35);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();

  w = u8g.getWidth();
  //hieght = u8g.getHeight();

  d = (w - u8g.getStrWidth(towerMenuValues[currentMenuItem])) / 2;
  u8g.drawStr(d, (h + 10), towerMenuValues[currentMenuItem]);
  //Serial.print("H >> "); Serial.println(h); Serial.print("W >> "); Serial.println(w); Serial.print("hieght >> "); Serial.println(hieght);
}

void drawFirstPage(void) {
  uint8_t i, h, th;
  u8g_uint_t w, d, hieght;

  u8g.setFont(u8g_font_6x12);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();
  u8g.setDefaultForegroundColor();

  h = u8g.getFontAscent() - u8g.getFontDescent();
  w = u8g.getWidth();
  hieght = u8g.getHeight();

  Serial.print("H >> "); Serial.println(h); Serial.print("W >> "); Serial.println(w); Serial.print("hieght >> "); Serial.println(hieght);

  for ( i = 0; i < MAIN_MENU_ITEMS; i++ ) {
    d = 1;
    th = 2;
    if (i == (MAIN_MENU_ITEMS - 1)) {
      d = (w - u8g.getStrWidth(mainMenuItems[i])) / 2;
      th = 4;
    }

    u8g.setDefaultForegroundColor();
    if ( i == currentMenuItem ) {
      u8g.drawBox(0, i * h + th, w, h);
      u8g.setDefaultBackgroundColor();
    }
    u8g.drawStr(d, i * h + th, mainMenuItems[i]);
    if (i != (MAIN_MENU_ITEMS - 1)) {
      u8g.drawStr(w - 15, i * h + th, mainMenuValues[i]);
    }
  }

  u8g.setDefaultForegroundColor();
  u8g.drawFrame(0, 0, w, 33);
  u8g.drawFrame(0, 34, w, 30);


  if (message2.length() > 0) {
    d = (w - u8g.getStrWidth(message1.c_str())) / 2;
    u8g.drawStr(d, 38, message1.c_str());
    d = (w - u8g.getStrWidth(message2.c_str())) / 2;
    u8g.drawStr(d, 49, message2.c_str());
  } else {

  }

}

void handleRotaryButton(void) {
  Serial.println("Rotary Button pressed >>>> ");
  if (1 == currentPageNumber) {
    if (0 == currentMenuItem) {
      currentPageNumber = 2;
      currentNumberOfMenuItems = PLATE_MENU_ITEMS;
      currentMenuItem = 0;
    } else if (1 == currentMenuItem) {
      currentPageNumber = 3;
      currentNumberOfMenuItems = TOWER_MENU_ITEMS;
      currentMenuItem = 0;
    } else if (2 == currentMenuItem) {
      if (!isHanoiInputReady) {
        isHanoiInputReady = true;
        numberOfPlates = atoi(mainMenuValues[0]);

        if ("B" == mainMenuValues[1]) {
          destinationTower = 1;
          intermediateTower = 2;
        } else if ("C" == mainMenuValues[1]) {
          destinationTower = 2;
          intermediateTower = 1;
        }

        Serial.println("numberOfPlates >>>> ");
        Serial.print(numberOfPlates);
        Serial.print(" destinationTower >>>> ");
        Serial.print(destinationTower);
        Serial.print(" intermediateTower >>>> ");
        Serial.print(intermediateTower);
      } else if (isTowerApositionReady && !isPlatesReady) {
        isPlatesReady = true;

        message1 = "Plate moving";
        message2 = "STARTED!";
        menu_redraw_required = 1;
      }
    }
  } else if (2 == currentPageNumber) {
    mainMenuValues[0] = plateMenuValues[currentMenuItem];
    currentPageNumber = 1;
    currentNumberOfMenuItems = MAIN_MENU_ITEMS;
    currentMenuItem = 0;
  } else if (3 == currentPageNumber) {
    mainMenuValues[1] = towerMenuValues[currentMenuItem];
    currentPageNumber = 1;
    currentNumberOfMenuItems = MAIN_MENU_ITEMS;
    currentMenuItem = 1;
  } else {
    mainMenuValues[0] = "3";
    mainMenuValues[1] = "B";
    currentPageNumber = 1;
    currentNumberOfMenuItems = MAIN_MENU_ITEMS;
    currentMenuItem = 0;
  }
  menu_redraw_required = 1;
}

void handleResetButton(void) {
  Serial.println("Reset Button pressed >>>> ");
  menu_redraw_required = 1;
}

void drawDisplay(void) {
  if (  menu_redraw_required != 0 ) {
    u8g.firstPage();
    do  {
      drawCurrentPage();
    } while ( u8g.nextPage() );
    menu_redraw_required = 0;
  }

  if (myInterruptVar > 1000) {
    myInterruptVar = 0;
  }

  if (rotary_button_pressd == 1) {
    handleRotaryButton();
    rotary_button_pressd = 0;
  }

  if (reset_button_pressd == 1) {
    handleResetButton();
    reset_button_pressd = 0;
  }

  if (DialCount != PreDialCount) {
    //Serial.println("DialCount >>>> "); Serial.print(DialCount);
    PreDialCount = DialCount;
    currentMenuItem = DialCount % currentNumberOfMenuItems;
    menu_redraw_required = 1;
  }
}

ISR(TIMER3_COMPB_vect)
{
  if (!digitalRead(BUTTON_DIO) && rotary_button_check == 1) {
    rotary_button_pressd = 1;
    rotary_button_checked = myInterruptVar;
    rotary_button_check = 0;
  }

  if (!digitalRead(RESET_DIO) && reset_button_check == 1) {
    reset_button_pressd = 1;
    reset_button_checked = myInterruptVar;
    reset_button_check = 0;
  }


  if (abs(myInterruptVar - rotary_button_checked) > 70) {
    rotary_button_check = 1;
  }

  if (abs(myInterruptVar - reset_button_checked) > 70) {
    reset_button_check = 1;
  }

  DialPos = (digitalRead(ROT_EN_B) << 1) | digitalRead(ROT_EN_A);

  if (DialPos == 3 && Last_DialPos == 1)
  {
    /* If so increase the dial counter and display it */
    DialCount++;
  }

  /* Is the dial being turned counter-clockwise ? */
  if (DialPos == 3 && Last_DialPos == 2)
  {
    /* If so decrease the dial counter and display it */
    DialCount--;
  }

  Last_DialPos = DialPos;

  myInterruptVar++;
  //Serial.println("++++++++++++++++++++++++++++++++++++++++++");
}











