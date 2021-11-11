//Pinout.h
//RoboCup 2019 PCB Pinout:

#define INVERT 1

//Sensors

//Ground Sensor
#define GS_ANI A0
#define GS_S0 48
#define GS_S1 46
#define GS_S2 44
#define GS_S3 42
#define VLED 43
#define GS0 0
#define GS1 1
#define GS2 2
#define GS3 3
#define GS4 4
#define GS5 5
#define GS6 6
#define GS7 7
#define GS8 8
#define GS9 9
#define GS10 10
#define GS11 11
#define GS12 12
#define GS13 13
#define GS14 14
#define GS_THRESHOLD_WHITE 440 //Lower this //750 for first robot //575 at the lab //625 at competition robot2 //590 robot 1
#define GS_THRESHOLD_BLACK 365 //365 //Change this

//Compass Sensor - Serial
#define compassCommand 0x12 //8 bit mode - use 0x13 for 16 bit
#define startCalibration1 0xF0 //F0
#define startCalibration2 0xF5 //F5
#define startCalibration3 0xF7 //F7 for simple calibration //F6 for tilt-compensation
#define endCalibration 0xF8

// Current Sensing (ADD PIN #)
/*
  #define CS1 ?
  #define CS2 ?
  #define CS3 ?
  #define CS4 ?
*/

//Mechanical Control

//Motor Drivers
#define M4 1
#define M4_IN1 23
#define M4_IN2 25
#define M4_PWM 2

#define M3 2
#define M3_IN1 27
#define M3_IN2 29
#define M3_PWM 3

#define M1 3
#define M1_IN1 30
#define M1_IN2 32
#define M1_PWM 4

#define M2 4
#define M2_IN1 34
#define M2_IN2 36
#define M2_PWM 5

#define M5 5
#define M5_IN1 38
#define M5_IN2 40
#define M5_PWM 6

//UI/Debugging
//Buttons // (FIX PIN #) // 0 = pressed, 1 = not pressed
#define B1 35 // up
#define B2 33 // down
#define B3 31 // select


//Robot Communication
//Bluetooth Module - Serial2
