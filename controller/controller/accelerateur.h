#pragma once
#include <TimerOne.h> 
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>

#ifndef ACCELERATEUR_H
#define ACCELERATEUR_H


// Conversion ratio
#define A_R 16384.0
#define G_R 131.0
 
//Conversion from radians to degrees 180/PI
#define RAD_A_DEG = 57.295779
 


// Euler angles struct
struct eulerAngles6050 {
	float yaw;
	float pitch;
	float roll;
};


struct AccErrors {
	float AccErrorX;
	float AccErrorY;
	float GyroErrorY;
	float GyroErrorX;
	float GyroErrorZ;
};


class Accelerateur {
public:
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  float Acc[2];
  float Gy[2];
  float Angle[2];
  
	const int MPU = 0x69; // MPU6050 I2C address
	float AccX, AccY, AccZ;
	float GyroX, GyroY, GyroZ;
	float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
	float roll, pitch, yaw;

  float correct_yaw = 0;
  float correct_pitch = 0;
  float correct_roll = 0;
  

	float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

	float elapsedTime, currentTime, previousTime;
	float Tmp;
	int c;

  int this_time;
  int time_ant = 0;
  float interval;


	struct AccErrors eulerError;
	struct eulerAngles6050 euler;

	Accelerateur();
	struct eulerAngles6050 calculer_angles();
	struct AccErrors calculate_IMU_error();
	void initialize_I2C();
  void to_zero();

};

#endif 
