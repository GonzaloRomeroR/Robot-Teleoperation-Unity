
///////////////////////TIMER//////////////////////////////////////////////
#include <TimerOne.h> 



///////////////////////GIROSCOPE MPU 9250//////////////////////////////////////////////
bool start_timer = false;


#include "I2Cdev.h"

// Uncomment this code in order to use better MPU9250 code, but not working with two accelerometers
//#include "MPU6050_6Axis_MotionApps20.h"
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//    #include "Wire.h"
//#endif
//MPU6050 mpu;
//
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
//
//// MPU control/status vars
//bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
//uint8_t fifoBuffer[64]; // FIFO storage buffer
//
//// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//
//// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


#include "MPU9250.h"

MPU9250 mpu;



struct eulerAngles {
  float yaw = 0;
  float pitch = 0;
  float roll = 0;
};

eulerAngles ang_euler;

float callibrate_yaw = 0;
float callibrate_roll = 0;
float callibrate_pitch = 0;

bool callibration_done = false;

float aux_yaw;
//////////////////////////////////////////////////////////////////////////////////////

///////////////////////BLUETOOTH//////////////////////////////////////////////

#include <SoftwareSerial.h>// se importa libreria serial
SoftwareSerial tmserial (10, 11); //El pin 10 sera el Rx, y el pin 11 sera el Tx
String message;

//////////////////////////////////////////////////////////////////////////////

///////////////////////JOYSTICK//////////////////////////////////////////////

const int pinLED = 13;
const int pinJoyX = A0;
const int pinJoyY = A1;
const int pinJoyButton = 3;

struct Joystick {
  int X_value = 0;
  int Y_value = 0;
  int button_value = false;
};

Joystick joystick;

//////////////////////////////////////////////////////////////////////////////


///////////////////////GYROSCOPE MPU 6050/////////////////////////////////////
#include "accelerateur.h"
Accelerateur Accelerometer;
struct eulerAngles6050 eu;


//////////////////////////////////////////////////////////////////////////////
