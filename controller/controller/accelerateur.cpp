#include "accelerateur.h"

Accelerateur::Accelerateur() {}


void Accelerateur::initialize_I2C() {
  // Initialize accelerometer
  Wire.begin();                      // Initialiser communication
  Wire.beginTransmission(MPU);       // Commencer communication avec MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Ecrire dans le registre 6B
  Wire.write(0x00);                  // Faire reset - place a 0 dans le 6B registre
  Wire.endTransmission(true);        // Finir la transmition
}


struct eulerAngles6050 Accelerateur::calculer_angles() {
  // === Lire l'accelerometre === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Demander le registre 0x3B - correspond au AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // A partir du 0x3B, on demande 6 registres
  AcX = Wire.read() << 8 | Wire.read(); // Chaque valeur utilise 2 registres
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // A partir des valeurs du acelerometre, on calcule les angles Y, X
  // respectivement, avec la formule de la tangente.
  Acc[1] = atan(-1 * (AcX / A_R) / sqrt(pow((AcY / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;
  Acc[0] = atan((AcY / A_R) / sqrt(pow((AcX / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;

  // Lire les valeurs du Giroscope
  
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4, true); // A diference du Acelerometre, on demande seulement 4 registres
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();


  this_time = millis();
  interval = (this_time - time_ant) / 1000.0;
  

  // Calcule du angle du Giroscope (Valeurs Raw GyX, GyY divise G_R nous donne le valeur en °/s)
  Gy[0] = GyX / G_R;
  Gy[1] = GyY / G_R;

  // Utiliser un Filtre Complementaire
  Angle[0] = 0.95 * (Angle[0] + Gy[0] * interval) + 0.05 * Acc[0];
  Angle[1] = 0.95 * (Angle[1] + Gy[1] * interval) + 0.05 * Acc[1];

  time_ant = this_time;
  
  roll = Angle[0];
  pitch = Angle[1];
  yaw = Angle[0];

  euler.roll = Angle[0] - correct_roll;
  euler.pitch = 0;
  euler.yaw = 0;

  return euler;

}


struct AccErrors Accelerateur::calculate_IMU_error() {
  
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    // Sum all readings
    eulerError.AccErrorX = eulerError.AccErrorX + (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    eulerError.AccErrorY = eulerError.AccErrorY + (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  // Divide the sum by 200 to get the error value
  eulerError.AccErrorX = eulerError.AccErrorX / 200.0;
  eulerError.AccErrorY = eulerError.AccErrorY / 200.0;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  // Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor

  //
  //	eulerError.AccErrorX = AccErrorX;
  //	eulerError.AccErrorY = AccErrorY;
  eulerError.GyroErrorX = GyroErrorX;
  eulerError.GyroErrorY = GyroErrorY;
  eulerError.GyroErrorZ = GyroErrorZ;

  return eulerError;

}


void Accelerateur::to_zero() {

  correct_yaw = yaw;
  correct_roll = roll;
  correct_pitch = pitch;

}
