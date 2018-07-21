//This is taken from https://www.reddit.com/r/Dobot/comments/45ilan/controlling_dobot_with_ramps_14

#include <AccelStepper.h>
#include <MultiStepper.h>
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
#define TEMP_0_PIN          13
#define TEMP_1_PIN          14
#define zPlus             40
#define zMinus            42

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

int isMagnetOn = 0;
int** movesArray = NULL;
int numberOfMoves = 0;
int currentMove = -1;

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
  //XAxis.setSpeed(100);
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

void loop () {

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

  YAxis.moveTo(26181);
  XAxis.moveTo(2095);

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

  while (YAxis.distanceToGo() != 0 || XAxis.distanceToGo() != 0) {
    YAxis.run();
    XAxis.run();
  }





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
  int numberOfPlates = 6;
  numberOfMoves = pow(2, numberOfPlates) - 1;

  if (movesArray != NULL) {
    movesArray = (int**) realloc(movesArray, numberOfMoves * sizeof(int));
  } else {
    movesArray = (int**) malloc(numberOfMoves * sizeof(int));
  }

  for (int iRow = 0 ; iRow < numberOfMoves ; iRow++)
  {
    movesArray[iRow] = (int *)malloc(3 * sizeof(int));
  }

  //  for (int iRow = 0 ; iRow < numberOfMoves ; iRow++)
  //  {
  //    for (int iCol = 0 ; iCol < 3 ; iCol++)
  //    {
  //      movesArray[iRow][iCol] = 3;
  //    }
  //  }

  Serial.println("Hanoi :- ");
  Serial.println(numberOfPlates);

  currentMove = -1;
  towerOfHanoi(numberOfPlates, 0, 1, 2);

  for (int iRow = 0 ; iRow < numberOfMoves ; iRow++)
  {
    Serial.println(" ");
    Serial.print( movesArray[iRow][0]);Serial.print( movesArray[iRow][1]);Serial.print( movesArray[iRow][2]);
  }
}

// C recursive function to solve tower of hanoi puzzle
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

















