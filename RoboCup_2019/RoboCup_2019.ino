/*
   CodeRunners RoboCup Code for autonomous soccer-playing robots

   Season: 2019
   Members: Nadav Soudry, Aytan Geschwind, Noam Goldwasser, Adiel Benisty
   Lead Programmer: Nadav Soudry

   Code includes  offensive and defensive modes and test code for debugging
*/

// math library includes M_PI variable for angle calculations
#include <math.h>
#include <stdio.h>

// OLED screen for UI
#include <U8g2lib.h>
#include <U8x8lib.h>
#include <SPI.h>
U8G2_SSD1306_128X64_NONAME_2_HW_I2C OLED(U8G2_R0); // OLED definition

// Pixy Camera for computer vision
#include <Pixy.h>
Pixy pixy;
uint16_t blocks;

// location of pins for IO
#include "Pinout.h"

// Compass sensor
float fieldNorth = 0;
float rotOffset = 0; //Change this to point the robot at a different angle relative to fieldNorth
boolean compassWorking = true;

// White line variables
int groundSensor[15];
bool lineLeft = false;
bool lineRight = false;
bool lineFront = false;
bool lineBack = false;

// menu choice and switch statement
int choice = 0;     // show menu        // 0
char options[][15] = {"Offense",        // 1
                      "Defense",        // 2
                      "Camera Test",    // 3
                      "GS Test",        // 4
                      "GS Calib",       // 5
                      "CMPS Test",      // 6
                      "CMPS Calib",     // 7
                      "Dribbler Test",  // 8
                      "Capture Ball",   // 9
                      "Pursuit Angle",  // 10
                      "Return North"    // 11
                     };
int optionSize = 11;
int STATE = 0; // what the robot is currently doing

// ball pursuit
double acceptableDistanceToBall = 45; // might not need changing, should be arc length
double ballAngle;
double ballDistance;

// goals
double yellowAngle;
double yellowDistance;
double blueAngle;
double blueDistance;

// dribbler speed
int dribblerSpeed = 170;

void setup() {
  SerialUSB.begin(9600);
  Serial1.begin(9600);
  Serial.begin(9600);
  SerialUSB.println("Starting");

  // OLED setup
  OLED.begin();
  OLED.setFont(u8g2_font_helvB12_tr);
  OLED.setFontMode(1); //transparent
  OLED.setDrawColor(2); //shows white on black and black on white
  OLED.clearDisplay();

  // button setup
  pinMode(B1, INPUT);
  pinMode(B2, INPUT);
  pinMode(B3, INPUT);

  //Motor Setup
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);

  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  pinMode(M4_PWM, OUTPUT);

  pinMode(M5_IN1, OUTPUT);
  pinMode(M5_IN2, OUTPUT);
  pinMode(M5_PWM, OUTPUT);

  //Ground Sensor Setup
  pinMode(GS_ANI, INPUT);
  pinMode(GS_S0, OUTPUT); pinMode(GS_S1, OUTPUT); pinMode(GS_S2, OUTPUT); pinMode(GS_S3, OUTPUT);
  pinMode(VLED, OUTPUT);
  digitalWrite(VLED, HIGH);

  // pixy camera
  pixy.init();
  delay(1000);

  // displays different options for testing or playing.
  drawOptions(options, choice, optionSize);

  // set north when robot turns on to the value it is currently
  fieldNorth = readCompass();
}

void loop() {
  // switch statement for all states mid-playing
  // switch case will carry out a specific function based on the value of STATE
  switch (STATE)
  {
    case 0:
      // check buttons, update screen, update STATE and choice
      checkButtons();
      break;
    case 1:
      // offense
      STATE = 0;
      break;
    case 2:
      // defense
      STATE = 0;
      break;
    case 3:
      // camera test
      cameraTest();
      break;
    case 4:
      // ground sensor test
      groundSensorTest();
      break;
    case 5:
      // ground sensor calibration
      STATE = 0;
      break;
    case 6:
      // compass test
      compassTest();
      STATE = 0;
      break;
    case 7:
      // compass calibration
      compassCalibration();
      while (digitalRead(B3) == 0);
      STATE = 0;
      break;
    case 8:
      // dribbler test
      dribblerTest();
      break;
    case 9:
      // capture ball
      captureBall();
      break;
    case 10:
      pursuitAngle();
      break;
    case 11:
      returnNorth();
      break;
  }
}

/*
   Checks for the ball and updates "blocks"
   PROGRAM: update goal blocks
*/
void updatePixyObjects() {
  blocks = pixy.getBlocks();
  delay(20);
}

/*
   capture ball test code
*/
void captureBall() {
  // update blocks object
  delay(20);
  uint16_t blocks;
  blocks = pixy.getBlocks();

  if (blocks)
  {
    // distance angle formula
    int ballXCoord = pixy.blocks[0].x - 160;
    int ballYCoord = pixy.blocks[0].y - 100;
    ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
    ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);

    // make angle positive
    if (ballAngle < 0) {
      ballAngle += 360;
    }

    // shift angle by 85 degrees so north is 0
    ballAngle += 85;
    if (ballAngle >= 360) {
      ballAngle -= 360;
    }

    // double to string
    char angleString[6];
    char distanceString[6];
    dtostrf(ballAngle, 6, 2, angleString);
    dtostrf(ballDistance, 6, 2, distanceString);

    if ((ballAngle < 10 || ballAngle > 355) && ballDistance < 58) {
      brakeMotors();
      updateScreen("Captured Ball", distanceString, angleString);
    } else {
      updateScreen("Ball", distanceString, angleString);

      // drive to ball
      ballAngle = ballAngle + 180;
      drive(ballAngle, 0, 230);
    }
  } else {
    brakeMotors();
    updateScreen("No Object", "", "");
  }
}

/*
   functions to control motors:
   (1) control driving
   (2) control a single motor
   (3) break all motors
*/
#define POSITIVE true
#define NEGATIVE false
void drive(int dir, int rot, int PWR)
{
  //In order to make the math for x-configuration omnidrive simpler, the axes are shifted 45 degrees
  //Instead of using an x-axis for left/right and a y-axis for forward/back,
  //an alpha axis that points forward-right/backward-left and a beta axis that points forward-left/backward right are used.

  int alphaVec = 0; //this vector points forward-right
  int betaVec = 0; //this vector points forward-left

  dir =  dir - 90; // make 0 straight

  dir = dir - 45; //shifting axis
  if (dir < 0) {
    dir = dir + 360;
  }
  else if (dir >= 360) {
    dir = dir - 360;
  }

  int scale = 10000; //creates a range between -10,000 and 10,000 for alphaVec and betaVec

  float dirRad = (float)dir * 71 / 4068; //convert dir to radians

  //calculate alpha and beta vectors
  alphaVec = (float)(scale * cos(dirRad));
  betaVec = (float)(scale * sin(dirRad));

  //record the signs of alphaVec and betaVec for later use
  bool alphaSign;
  if (alphaVec >= 0)
  {
    alphaSign = POSITIVE;
  }
  else
  {
    alphaSign = NEGATIVE;
  }
  bool betaSign;
  if (betaVec >= 0)
  {
    betaSign = POSITIVE;
  }
  else
  {
    betaSign = NEGATIVE;
  }
  //remove the signs from alpha and betaVec
  alphaVec = abs(alphaVec);
  betaVec = abs(betaVec);
  //Create a maximum value based on the larger of the two vectors (in magnitude, without signs)
  int max;
  if (alphaVec >= betaVec)
  {
    max = alphaVec;
  }
  else
  {
    max = betaVec;
  }
  //map vectors using the max value obtained and the given motor PWR
  alphaVec = map(alphaVec, 0, max, 0, PWR);
  betaVec = map(betaVec, 0, max, 0, PWR);
  //put the signs back
  if (alphaSign == NEGATIVE)
  {
    alphaVec = -(alphaVec);
  }
  if (betaSign == NEGATIVE)
  {
    betaVec = -(betaVec);
  }

  //Send Commands to motors
  motor(M1, betaVec + rot);
  motor(M2, alphaVec + rot);
  motor(M3, betaVec - rot);
  motor(M4, alphaVec - rot);
}

void motor(int motorID, int PWR) //produces low level signals to each individual motor drivers
{
  bool motorDirection;
  //Checking for valid PWR value
  if (PWR <= 255 && PWR >= 0) {
    motorDirection = true;
  }
  else if (PWR >= -255 && PWR < 0) {
    motorDirection = false;
    PWR = -(PWR);
  }
  else {
    Serial.println("Invalid motor power");
    return; //error
  }
  //Handling Inverted motor control for second robot
  if (INVERT)
  {
    if (motorDirection == true)
    {
      motorDirection = false;
    }
    else
    {
      motorDirection = true;
    }
  }

  //Selecting a motor to run
  if (motorID == M1)
  {
    if (motorDirection == true) {
      digitalWrite(M1_IN1, LOW);
      digitalWrite(M1_IN2, HIGH);
    }
    else {
      digitalWrite(M1_IN1, HIGH);
      digitalWrite(M1_IN2, LOW);
    }
    analogWrite(M1_PWM, PWR);
  }
  else if (motorID == M2)
  {
    if (motorDirection == true) {
      digitalWrite(M2_IN1, LOW);
      digitalWrite(M2_IN2, HIGH);
    }
    else {
      digitalWrite(M2_IN1, HIGH);
      digitalWrite(M2_IN2, LOW);
    }
    analogWrite(M2_PWM, PWR);
  }
  else if (motorID == M3)
  {
    if (motorDirection == true) {
      digitalWrite(M3_IN1, LOW);
      digitalWrite(M3_IN2, HIGH);
    }
    else {
      digitalWrite(M3_IN1, HIGH);
      digitalWrite(M3_IN2, LOW);
    }
    analogWrite(M3_PWM, PWR);
  }
  else if (motorID == M4)
  {
    if (motorDirection == true) {
      digitalWrite(M4_IN1, LOW);
      digitalWrite(M4_IN2, HIGH);
    }
    else {
      digitalWrite(M4_IN1, HIGH);
      digitalWrite(M4_IN2, LOW);
    }
    analogWrite(M4_PWM, PWR);
  }
  else if (motorID == M5)
  {
    if (motorDirection == true) {
      digitalWrite(M5_IN1, LOW);
      digitalWrite(M5_IN2, HIGH);
    }
    else {
      digitalWrite(M5_IN1, HIGH);
      digitalWrite(M5_IN2, LOW);
    }
    analogWrite(M5_PWM, PWR);
  }
}
void brakeMotors()
{
  motor(1, 0);
  motor(2, 0);
  motor(3, 0);
  motor(4, 0);
  motor(5, 0);
}

/*
   test mode for the ground sensor
*/
void groundSensorTest() {
  // updateScreen("GS Test", "", "");

  lineLeft = false;
  lineRight = false;
  lineFront = false;
  lineBack = false;

  readGroundSensor();

  SerialUSB.println(groundSensor[GS0]);

  //Serial.println(groundSensor[GS0]);
  if (groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)
  {
    lineFront = true;
    updateScreen("GS Test", "front", "");
  }
  if (groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)
  {
    lineRight = true;
    updateScreen("GS Test", "", "right");
  }
  if (groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)
  {
    lineLeft = true;
    updateScreen("GS Test", "", "left");
  }
  if (groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)
  {
    lineBack = true;
    updateScreen("GS Test", "back", "");
  }

  if (!lineLeft && !lineRight && !lineFront && !lineBack) {
    updateScreen("GS Test", "no line", "");
  }

  while (digitalRead(B3) == 0) {
    STATE = 0;
  }
}

/*
   Ground sensor checking
*/
void readGroundSensor()
{
  for (int i = 0; i < 15; i++)
  {
    groundSensor[i] = readGSMux(i);
  }
}
int readGSMux(int channel) //selects the given channel on the ground sensor multiplexor and reads the analog signal
{
  switch (channel) { //Uses truth table in multiplexor datasheet for selecting channels: http://www.mouser.com/ds/2/405/cd74hc4067-441121.pdf
    case 0:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, LOW);
      break;
    case 1:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, LOW);
      break;
    case 2:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, LOW);
      break;
    case 3:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, LOW);
      break;
    case 4:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, LOW);
      break;
    case 5:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, LOW);
      break;
    case 6:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, LOW);
      break;
    case 7:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, LOW);
      break;
    case 8:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, HIGH);
      break;
    case 9:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, HIGH);
      break;
    case 10:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, HIGH);
      break;
    case 11:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, HIGH);
      break;
    case 12:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, HIGH);
      break;
    case 13:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, HIGH);
      break;
    case 14:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, HIGH);
      break;
    case 15:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, HIGH);
      break;
  }
  delayMicroseconds(2); //Give time for the multiplexor to switch - can change to 1 microsecond
  int analogInput = analogRead(GS_ANI);
  return analogInput;
}

void cameraTest() {
  int selectedObject = 0; // 0 = ball, 1 = yellow goal, 2 = blue goal

  while (!(digitalRead(B3) == 0)) {
    // update blocks object
    delay(20);
    uint16_t blocks;
    blocks = pixy.getBlocks();

    if (blocks)
    {
      if (selectedObject == 0) {
        // distance angle formula
        int ballXCoord = pixy.blocks[0].x - 160;
        int ballYCoord = pixy.blocks[0].y - 100;
        
        ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
        ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);
        
        // make angle positive
        if (ballAngle < 0) {
          ballAngle += 360;
        }

        // shift angle by 85 degrees so north is 0
        ballAngle += 85;
        if (ballAngle >= 360) {
          ballAngle -= 360;
        }

        // double to string
        char angleString[6];
        char distanceString[6];
        dtostrf(ballAngle, 6, 2, angleString);
        dtostrf(ballDistance, 6, 2, distanceString);

        if ((ballAngle < 10 || ballAngle > 355) && ballDistance < 58) {
          updateScreen("Captured Ball", distanceString, angleString);
        } else {
          updateScreen("Ball", distanceString, angleString);
        }
      }
      else if (selectedObject == 1) {
        // distance angle formula
        int yellowXCoord = pixy.blocks[1].x - 160;
        int yellowYCoord = pixy.blocks[1].y - 100;
        yellowAngle = atan2(yellowYCoord, yellowXCoord) * 180 / 3.14;
        yellowDistance = sqrt(yellowYCoord * yellowYCoord + yellowXCoord * yellowXCoord);

        // make angle positive
        if (yellowAngle < 0) {
          yellowAngle += 360;
        }

        // shift angle by 79 degrees so north is 0
        yellowAngle += 79;
        if (yellowAngle >= 360) {
          yellowAngle -= 360;
        }

        // double to string
        char angleString[6];
        char distanceString[6];
        dtostrf(ballAngle, 6, 2, angleString);
        dtostrf(ballDistance, 6, 2, distanceString);
        updateScreen("yellow goal", distanceString, angleString);
      }
      else if (selectedObject == 2) {
        // distance angle formula
        int blueXCoord = pixy.blocks[2].x - 160;
        int blueYCoord = pixy.blocks[2].y - 100;
        blueAngle = atan2(blueYCoord, blueXCoord) * 180 / 3.14;
        blueDistance = sqrt(blueYCoord * blueYCoord + blueXCoord * blueXCoord);

        // make angle positive
        if (blueAngle < 0) {
          blueAngle += 360;
        }

        // shift angle by 58 degrees so north is 0
        blueAngle += 58;
        if (blueAngle >= 360) {
          blueAngle -= 360;
        }

        // double to string
        char angleString[6];
        char distanceString[6];
        dtostrf(blueAngle, 6, 2, angleString);
        dtostrf(blueDistance, 6, 2, distanceString);
        updateScreen("blue goal", distanceString, angleString);
      }
    } else {
      updateScreen("No Object", "", "");
    }

    // check buttons
    // when the down button is pressed
    if (digitalRead(B2) == 0)
    {
      selectedObject--;
      if (selectedObject == -1)
      {
        selectedObject = 2;
      }
      while (digitalRead(B2) == 0); // only switches when the button is released.
    }

    // when the up button is pressed.
    if (digitalRead(B1) == 0)
    {
      selectedObject++;
      if (selectedObject == 3)
      {
        selectedObject = 0;
      }
      while (digitalRead(B2) == 0); // only switches when the button is released.
    }
  }

  STATE = 0;
  while (digitalRead(B3) == 0);
}

/*
    OLED controls:
    (1) updateScreen("line 1", "line 2", "line 3")
    (2) OLED.clearDisplay() clears screen (built in)
*/
void updateScreen(char input1[], char input2[], char input3[])
{
  OLED.firstPage();
  do
  {
    updateScreenHelper(input1, 0);
    updateScreenHelper(input2, 1);
    updateScreenHelper(input3, 2);
  } while (OLED.nextPage());
}

void drawOptions(char optionArray[][15], int selection, int numOptions)
{
  // OLED printing
  OLED.firstPage();
  do {
    OLED.drawBox(2, 22, 120, 18);
    if (selection == 0)
    {
      updateScreenHelper(options[numOptions - 1], 0);
    }
    else
    {
      updateScreenHelper(options[selection - 1], 0);
    }
    updateScreenHelper(options[selection], 1);
    if (selection == numOptions - 1)
    {
      updateScreenHelper(options[0], 2);
    }
    else
    {
      updateScreenHelper(options[selection + 1], 2);
    }
  } while (OLED.nextPage());
}
void updateScreenHelper(char input[], int line)
{
  //OLED.setCursor(2, 15 + (20 * line));
  OLED.drawStr(2, 15 + (20 * line), input);
}

/*
   Check the buttons and update variables
   Updates screen if up or down is selected
*/
void checkButtons () {
  // when the select button is pressed
  if (digitalRead(B3) == 0)
  {
    while (digitalRead(B3) == 0);
    STATE = choice + 1; // STATE only changes when the button is pressed.
  }

  // when the down button is pressed
  if (digitalRead(B2) == 0)
  {
    choice++; // the choice value increases by one
    if (choice == optionSize) // the option returns to itself
    {
      choice = 0;
    }
    while (digitalRead(B2) == 0); // only switches when the button is released.

    // displays different options for testing or playing.
    drawOptions(options, choice, optionSize);
  }

  // when the up button is pressed.
  if (digitalRead(B1) == 0)
  {
    choice--; // choice value decreases by one
    if (choice == -1)
    {
      choice = optionSize - 1;
    }
    while (digitalRead(B1) == 0);

    // displays different options for testing or playing.
    drawOptions(options, choice, optionSize);
  }
}

/*
   display values from the compass to determine if calibration is necessary
*/
void compassTest() {
  while (!(digitalRead(B3) == 0)) {
    float compassHeading = readCompass();
    float headDif = headingDif();
    if (headDif < -180) {
      headDif += 360;
    }
    headDif = -headDif;

    if (compassWorking == true)
    {
      // double to string
      char compassHeadingString[6];
      char compassHeadingDifString[6];
      dtostrf(compassHeading, 6, 2, compassHeadingString);
      dtostrf(headDif, 6, 2, compassHeadingDifString);

      updateScreen("Compass:", "", compassHeadingDifString);  // compassHeadingString
    }
    else
    {
      updateScreen("Compass:", "Not Found", "");
    }
    SerialUSB.print("Compass Heading: ");
    SerialUSB.println(compassHeading);
  }
  while (!(digitalRead(B3) == 0));
}
float headingDif() {
  float dif = readCompass() - fieldNorth;
  if (dif > 180) {
    dif = dif - 360;
  }
  return dif;
}

/*
   calibrate the compass
*/
void compassCalibration()
{
  updateScreen("CMPS_Cal", "Start   ", "");

  brakeMotors();

  SerialUSB.println("Press to Start then rotate Robot 360 Degrees to Calibrate");
  //Run Calibration Sequence
  //send calibration start sequence and recieve confirmation bytes
  byte confirm1;
  byte confirm2;
  byte confirm3;
  Serial.write(startCalibration1);
  delay(10);
  if (Serial.available()) {
    confirm1 = Serial.read();
  }
  else {
    SerialUSB.println("Error - compass not available");
    updateScreen("CMPS_Cal", "Start   ", "error");
    return;
  }
  delay(100);
  Serial.write(startCalibration2);
  delay(10);
  if (Serial.available()) {
    byte confirm2 = Serial.read();
  }
  else
  {
    SerialUSB.println("Error - compass not available");
    updateScreen("CMPS_Cal", "Start   ", "error");
    return;
  }
  delay(100);
  Serial.write(startCalibration3);
  delay(10);
  if (Serial.available()) {
    confirm3 = Serial.read();
  }
  else {
    updateScreen("CMPS_Cal", "", "error");
    return;
  }
  if (confirm1 != 0x55 || confirm2 != 0x55 || confirm3 != 0x55)
  {
    SerialUSB.println("Error - Wrong confirmation code");
    SerialUSB.println(confirm1);
    SerialUSB.println(confirm2);
    SerialUSB.println(confirm3);
    updateScreen("CMPS_Cal", "", "error");
  }
  delay(100);

  byte confirm4;
  SerialUSB.println("Press Button to finish when you cannot get further LED flashes");
  updateScreen("CMPS_Cal", "Start   ", "rotate");

  delay(500);

  while (digitalRead(B3) == HIGH) {
  }
  Serial.write(endCalibration);
  delay(5);
  if (Serial.available()) {
    confirm4 = Serial.read();
  }
  else
  {
    SerialUSB.println("Error - compass not available");
    return;
  }
  SerialUSB.println("Calibration Complete");
  updateScreen("CMPS_Cal", "complete.   ", "");
  delay(1000);
  return;
}

/*
   Return Compass data
*/
float readCompass() //8 bit mode to avoid errors
{
reset:
  Serial.write(compassCommand);
  long waitStartTime = millis();
  while (Serial.available() < 1)
  {
    long currentTime = millis();
    if ((currentTime - waitStartTime) > 400)
    {
      SerialUSB.println("No Compass Plugged In");
      compassWorking = false;
      return -1;
    }
  }
  if (compassWorking == false) //The compass wasn't working previously but it came back on
  {
    //Reset Serial communication
    SerialUSB.println("Compass Reset");
    Serial.end();
    Serial.begin(9600);
    compassWorking = true;
    //Serial.print("Number of Bytes Available: ");
    //Serial.println(Serial.available());
    goto reset; //restart function
  }
  byte compass_raw = Serial.read();
  int mappedVal = map(compass_raw, 0, 255, 0, 3600); //maps the 8bit angle to 0-3600
  float processedVal = ((float)mappedVal) / (float)10;
  return processedVal;
}

double pursuitAngle()
{
  // update blocks object
  delay(20);
  uint16_t blocks;
  blocks = pixy.getBlocks();

  if (blocks)
  {
    // distance angle formula
    int ballXCoord = pixy.blocks[0].x - 160;
    int ballYCoord = pixy.blocks[0].y - 100;
    ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
    ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);
    // make angle positive
    if (ballAngle < 0) {
      ballAngle += 360;
    }

    // shift angle by 85 degrees so north is 0
    ballAngle += 85;
    if (ballAngle >= 360) {
      ballAngle -= 360;
    }

    char ballAngleString[6];
    dtostrf(ballAngle, 6, 2, ballAngleString);

    double idealAngleFromBall = (acceptableDistanceToBall / ballDistance) * (180 / M_PI);
    double idealAngle1 = ballAngle + idealAngleFromBall;
    double idealAngle2 = ballAngle - idealAngleFromBall;

    if (ballAngle < 10 || ballAngle > 350) // straight ahead
    {
      drive(ballAngle + 180, 0, 240);
      // updateScreen("Pursuit Angle", ballAngleString, ballAngleString);
    }
    else if (ballAngle < 180) // on the left
    {
      // char pursuitAngleString[6];
      // dtostrf(idealAngle1, 6, 2, pursuitAngleString);

      // updateScreen("Pursuit Angle", ballAngleString, pursuitAngleString);
      // return idealAngle1;
      drive(idealAngle1 + 180, 0, 235);
    }
    else // on the right
    {
      // char pursuitAngleString[6];
      // dtostrf(idealAngle2, 6, 2, pursuitAngleString);

      // updateScreen("Pursuit Angle", ballAngleString, pursuitAngleString);
      // return idealAngle2;

      drive(idealAngle2 + 180, 0, 235);
    }

    delay(20);

    // stay pointing north
    float headDif;
    headDif = headingDif();
    if (headDif > 10) {
      drive(-20, constrain(headDif, 100, 150), 100);
    } else if (headDif < -10) {
      drive(20, constrain(5 * headDif, -150, -100), 100);
    }
  } else {
    brakeMotors();
  }
}

void returnNorth() {
  float headDif;
  headDif = headingDif();

  if (headDif > 10) {
    drive(-20, constrain(headDif, 100, 150), 100);
  } else if (headDif < -10) {
    drive(20, constrain(5 * headDif, -150, -100), 100);
  } else {
    brakeMotors();
  }

  // exit mode
  while ((digitalRead(B3) == 0)) {
    STATE = 0;
  }
}

// test dribbler motor and mechanism
void dribblerTest() {
  while (!(digitalRead(B3) == 0)) {
    // spin the dribbler with the dribblerSpeed speed
    motor(5, dribblerSpeed);

    if (digitalRead(B2) == 0) // decrease speed if down button is pressed
    {
      if (dribblerSpeed == 60) {
        dribblerSpeed = -61;
      } else if (dribblerSpeed > -255) // don't go bellow -255
      {
        dribblerSpeed--;
      }
    }

    if (digitalRead(B1) == 0) // increase speed if up button is pressed
    {
      if (dribblerSpeed == -60) {
        dribblerSpeed = 61;
      } else if (dribblerSpeed < 255) // don't go above 255
      {
        dribblerSpeed++;
      }
    }

    // update screen info
    char speedString[6];
    dtostrf((double)dribblerSpeed, 6, 2, speedString);
    updateScreen("speed up", speedString, "speed down");
  }

  while ((digitalRead(B3) == 0));
  STATE = 0;
  brakeMotors();
}

//dtostrf
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  asm(".global _printf_float");

  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}
