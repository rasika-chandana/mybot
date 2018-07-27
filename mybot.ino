//This is taken from https://www.reddit.com/r/Dobot/comments/45ilan/controlling_dobot_with_ramps_14

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include "U8glib.h"
#include <QueueArray.h>


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
static const double Z_MOTOR_STEPS = 51109.33;

static const double X_INIT_ANGLE = 24.1;
static const double Y_INIT_ANGLE = 11.8;


U8GLIB_ST7920_128X64_1X u8g(23, 17, 16);  // SPI Com: SCK = en = 18, MOSI = rw = 16, CS = di = 17

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

int numberOfPlates = 2;

int** movesArray = NULL;
int numberOfMoves = 0;
int currentMove = -1;
int currentTowerStatus[] = {0, 0, 0};

boolean isPlatesReady = true;
boolean isReadyForNextMove = true;
boolean isCurrentInstructionComplete = true;

int plateHieght = 15;
int plateFreeMoveGap = 5;
int baseX = 409;
int baseY = -216;
double baseZAngle = 23;

int isMagnetOn = 0;

long currentYsteps = 0;
long currentXsteps = 0;

long nextYsteps = 0;
long nextXsteps = 0;

static const int INST_TYPE_UP_AND_DOWN = 0;
static const int INST_TYPE_UP_ROTATE = 1;
static const int INST_TYPE_UP_MAGNET = 2;

typedef struct {
  int      type;
  long     x;
  long     y;
  long     z;
  boolean  magnetStatus;
} Instruction;

QueueArray <Instruction> queue;

void setup() {
  // flip screen, if required
  // u8g.setRot180();

  // set SPI backup if required
  //u8g.setHardwareBackup(u8g_backup_avr_spi);

  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255, 255, 255);
  }

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


  // draw();
  homeYAxis();
  homeXAxis();
  homeZAxis();

  //gotoInitYPosition();
  //homeXAxis();
  gotoInitZPosition();

  XAxis.setCurrentPosition(0);
  YAxis.setCurrentPosition(0);
  ZAxis.setCurrentPosition(0);

  XAxis.setMaxSpeed(10000.0);
  YAxis.setMaxSpeed(20000.0);
  ZAxis.setMaxSpeed(5000.0);

  XAxis.setAcceleration(1000.0);
  YAxis.setAcceleration(1000.0);
  ZAxis.setAcceleration(500.0);

  hanoiNoOfMoves();
  goToStartPoint();
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

  ZAxis.moveTo(-20820);

  while (ZAxis.distanceToGo() != 0) {
    ZAxis.run();
  }

  ZAxis.setCurrentPosition(0);
}

void goToStartPoint(void) {
  ZAxis.moveTo(-(stepsZ(baseZAngle)));

  while (ZAxis.distanceToGo() != 0) {
    ZAxis.run();
  }

  ZAxis.setCurrentPosition(0);

  long initY = baseY + (numberOfPlates * plateHieght) + plateFreeMoveGap;
  long initX = 409;

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


void draw(void) {
  // graphic commands to redraw the complete screen should be placed here
  u8g.setFont(u8g_font_courR08);
  u8g.drawStr( 0, 7, "The Bot is Ready");
  u8g.drawStr( 0, 15, "Rasika!");

  //  char buf[9];
  //  sprintf (buf, "%d", direction);
  //  u8g.drawStr(18, 20, buf);
  //  sprintf (buf, "%d", distanceToGo);
  //  u8g.drawStr(18, 28, buf);

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


void subMoveStepCommon(int fromTower, int toTower) {
  int towerDiff = toTower - fromTower;

  if (towerDiff < 0) {
    if (towerDiff == -1) {
      double highestTowerY =  baseY + plateFreeMoveGap + (currentTowerStatus[fromTower - 1] * plateHieght);
      double toTowerX = baseX / cos(rad(baseZAngle));
      addUpAndDownInstruction(toTowerX, highestTowerY);

      long zSteps = -(stepsZ(baseZAngle));
      addLeftRightInstruction(zSteps);
      // up plate hieght
      // move to right (-)  * 23
    } else {
      double highestTowerY =  baseY + plateFreeMoveGap + ((currentTowerStatus[fromTower - 1] > currentTowerStatus[fromTower - 2]) ? (currentTowerStatus[fromTower - 1] * plateHieght) : (currentTowerStatus[fromTower - 2] * plateHieght));
      double toTowerX = baseX / cos(rad(baseZAngle));
      addUpAndDownInstruction(toTowerX, highestTowerY);

      long zSteps = -(stepsZ(baseZAngle * 2));
      addLeftRightInstruction(zSteps);
      // up plate hieght
      // move to right (-)  * 23 * 2
    }
  } else if (towerDiff > 0) {
    if (towerDiff == 1) {
      double highestTowerY =  baseY + plateFreeMoveGap + (currentTowerStatus[fromTower + 1] * plateHieght);
      double toTowerX = baseX / cos(rad(baseZAngle));
      addUpAndDownInstruction(toTowerX, highestTowerY);

      long zSteps = stepsZ(baseZAngle);
      addLeftRightInstruction(zSteps);
      // up plate hieght
      // move to left  * 23
    } else {
      double highestTowerY =  baseY + plateFreeMoveGap + ((currentTowerStatus[fromTower + 1] > currentTowerStatus[fromTower + 2]) ? (currentTowerStatus[fromTower + 1] * plateHieght) : (currentTowerStatus[fromTower + 2] * plateHieght));
      double toTowerX = baseX / cos(rad(baseZAngle));
      addUpAndDownInstruction(toTowerX, highestTowerY);

      long zSteps = stepsZ(baseZAngle * 2);
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

  subMoveStepCommon( fromTower,  toTower);

  fromTowerX = baseX;
  fromTowerY = baseY + (currentTowerStatus[toTower] * plateHieght);
  addUpAndDownInstruction(fromTowerX, fromTowerY);
  addMagnetInstruction(false);
}


void loop () {

  if (isPlatesReady && (currentMove < numberOfMoves)) {
    if (isReadyForNextMove) {
      currentMove++;

      Serial.println("Current Move: ");
      Serial.print("A1 rad : "); Serial.print(currentMove);

      int fromTower = movesArray[currentMove][1];
      int toTower = movesArray[currentMove][2];
      int currentTower = 0;
//      if (currentMove > 0) {
//        currentTower = movesArray[currentMove - 1][2];
//      }

      if (currentTower == fromTower) {
        subMoveStep_2(fromTower, toTower);
        currentTower = toTower;
      } else {
        subMoveStepCommon(currentTower, fromTower);
        currentTower = fromTower;
        subMoveStep_2(fromTower, toTower);
        currentTower = toTower;
      }

      isReadyForNextMove = false;
    }

    Instruction instruction;

    if (!queue.isEmpty () && isCurrentInstructionComplete) {
      instruction = queue.dequeue ();

      if (instruction.type == INST_TYPE_UP_AND_DOWN) {
        Serial.println("Current Instruction : UP_AND_DOWN"); Serial.print(" X "); Serial.print(instruction.x); Serial.print(" Y"); Serial.print(instruction.y);
        isCurrentInstructionComplete = false;
      } else if (instruction.type == INST_TYPE_UP_ROTATE) {
        Serial.println("Current Instruction : ROTATE");Serial.print(" Z "); Serial.print(instruction.z);
        isCurrentInstructionComplete = false;
      } else if (instruction.type == INST_TYPE_UP_MAGNET) {
        Serial.println("Current Instruction : MAGNET");
        if (instruction.magnetStatus) {
          digitalWrite(HEATER_0_PIN, HIGH);

        } else {
          digitalWrite(HEATER_0_PIN    , LOW);
        }
        isCurrentInstructionComplete = true;
      }
    } else {
      isReadyForNextMove = true;
    }

    if (YAxis.distanceToGo() != 0) {
      YAxis.run();
    }

    if (XAxis.distanceToGo() != 0) {
      XAxis.run();
    }

    if (YAxis.distanceToGo() == 0 && XAxis.distanceToGo() == 0 && instruction.type == INST_TYPE_UP_AND_DOWN) {
      isCurrentInstructionComplete = true;
      XAxis.setCurrentPosition(0);
      YAxis.setCurrentPosition(0);
    }

    if (ZAxis.distanceToGo() != 0) {
      ZAxis.run();
    }

    if (ZAxis.distanceToGo() == 0 && instruction.type == INST_TYPE_UP_ROTATE) {
      isCurrentInstructionComplete = true;
      ZAxis.setCurrentPosition(0);
    }


    // add robot movements here with above queue

  } else {
    Serial.println("FINISHED!");
    // todo call init procedure
  }

  // add input read for display as well

  /**
    delay(3000);

    ZAxis.moveTo(4259);

    while (ZAxis.distanceToGo() != 0) {
      ZAxis.run();
    }
    ZAxis.setCurrentPosition(0);

    delay(3000);


    ZAxis.moveTo(-8518);

    while (ZAxis.distanceToGo() != 0) {
      ZAxis.run();
    }
    ZAxis.setCurrentPosition(0);

    delay(3000);



    ZAxis.moveTo(4259);

    while (ZAxis.distanceToGo() != 0) {
      ZAxis.run();
    }
    ZAxis.setCurrentPosition(0);

    delay(3000);
  **/


  //================ Based on R and D values ====================

  //  YAxis.moveTo(26096);
  //  XAxis.moveTo(2029);

  //  YAxis.moveTo(25086);
  //  XAxis.moveTo(2067);

  //  YAxis.moveTo(24106);
  //  XAxis.moveTo(2154);

  //  YAxis.moveTo(23150);
  //  XAxis.moveTo(2288);

  //  YAxis.moveTo(22218);
  //  XAxis.moveTo(2464);

  //  YAxis.moveTo(21307);
  //  XAxis.moveTo(2681);

  //================ Based on realy step calculation ===================

  //  YAxis.moveTo(26181);
  //  XAxis.moveTo(2095);

  //  YAxis.moveTo(25168);
  //  XAxis.moveTo(2135);

  //  YAxis.moveTo(24184);
  //  XAxis.moveTo(2225);

  //  YAxis.moveTo(23226);
  //  XAxis.moveTo(2363);

  //  YAxis.moveTo(22291);
  //  XAxis.moveTo(2545);

  //  YAxis.moveTo(21377);
  //  XAxis.moveTo(2769);

  //================ Based on realy step calculation ===================

  //  YAxis.moveTo(26181);
  //  XAxis.moveTo(2095);

  //  YAxis.moveTo(25243);
  //  XAxis.moveTo(2119);

  //  YAxis.moveTo(24184);
  //  XAxis.moveTo(2225);

  //  YAxis.moveTo(23226);
  //  XAxis.moveTo(2363);

  //  YAxis.moveTo(22291);
  //  XAxis.moveTo(2545);

  //  YAxis.moveTo(21377);
  //  XAxis.moveTo(2769);

  //  while (YAxis.distanceToGo() != 0 || XAxis.distanceToGo() != 0) {
  //    YAxis.run();
  //    XAxis.run();
  //  }





  /**

    XAxis.setMaxSpeed(10000.0);
    YAxis.setMaxSpeed(20000.0);
    XAxis.setAcceleration(1000.0);
    YAxis.setAcceleration(1000.0);

    delay(3000);

    XAxis.setCurrentPosition(0);
    YAxis.setCurrentPosition(0);

    YAxis.moveTo(26126);
    XAxis.moveTo(2000);

    while (YAxis.distanceToGo() != 0 || XAxis.distanceToGo() != 0) {
      YAxis.run();
      XAxis.run();
    }

    delay(1000);

    if (isMagnetOn == 1) {
      digitalWrite(HEATER_0_PIN    , LOW);
      isMagnetOn = 0;
    } else {
      digitalWrite(HEATER_0_PIN    , HIGH);
      isMagnetOn = 1;
    }


    if (YAxis.distanceToGo() == 0) {
      YAxis.setCurrentPosition(0);
      YAxis.moveTo(-26125);
    }

    if (XAxis.distanceToGo() == 0) {
      XAxis.setCurrentPosition(0);
      XAxis.moveTo(-1999);
    }

    delay(1000);

    while (YAxis.distanceToGo() != 0 || XAxis.distanceToGo() != 0) {
      YAxis.run();
      XAxis.run();
    }

  **/
}


void hanoiNoOfMoves(void) {

  numberOfMoves = pow(2, numberOfPlates) - 1;

  if (movesArray != NULL) {
    movesArray = (int**) realloc(movesArray, numberOfMoves * sizeof(int));
  } else {
    movesArray = (int**) malloc(numberOfMoves * sizeof(int));
  }

  for (int iRow = 0 ; iRow < numberOfMoves ; iRow++)
  {
    movesArray[iRow] = (int *) malloc(3 * sizeof(int));
  }

  Serial.println("Hanoi :- ");
  Serial.println(numberOfPlates);

  currentMove = -1;
  towerOfHanoi(numberOfPlates, 0, 1, 2);

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

  Serial.println(" ");
  Serial.print("A1 rad : "); Serial.print(A1); Serial.print("  A1 deg : "); Serial.print(backArmAngle); Serial.print("  Y move angle : "); Serial.print(backArmMoveAngle);
  Serial.print("  Y steps : "); Serial.print(backArmSteps);
  Serial.println(" ");
  Serial.print("A2 rad : "); Serial.print(A2); Serial.print("  A2 deg : "); Serial.print(foreArmAngle); Serial.print("  X move angle : "); Serial.print(foreArmMoveAngle);
  Serial.print("  X steps : "); Serial.print(foreArmSteps);
}

long stepsZ(double zAxixAngle) {
  long zSteps = round(Z_MOTOR_STEPS / 360 * zAxixAngle);
  Serial.println(" ");
  Serial.print("Z steps : "); Serial.print(zSteps);
  return zSteps;
}










