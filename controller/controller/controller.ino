// Program to send data from and gyroscope and a joystick to and application
// using a HC05 bluetooth module and an IMU MPU6050.

// Connection diagram attached

#include "variables.h"


Joystick get_joystick(Joystick joy, int pinX, int pinY, int pinButton) {
  int offset_X = -2;
  int offset_Y = 21;
  joy.X_value = map(analogRead(pinX) + offset_X, 0, 1023, -512, 512);
  joy.Y_value = map(analogRead(pinY) + offset_Y, 0, 1023, -512, 512);

  joy.button_value = digitalRead(pinButton);
  return joy;
}

void gyroscope_setup() {

  mpu.setup();
  //mpu.calibrateAccelGyro();
  //mpu.calibrateMag();

  //mpu.printCalibration();
}



struct eulerAngles gyroscope_read() {

  mpu.update();

  aux_yaw = mpu.getYaw() - callibrate_yaw;
  if (aux_yaw < -300){
    aux_yaw = aux_yaw + 360;
  }
  ang_euler.yaw = aux_yaw;
  ang_euler.roll = mpu.getRoll() - callibrate_roll;
  ang_euler.pitch = mpu.getPitch() - callibrate_pitch;

  return ang_euler;
}


void setTimer2() {
  cli();
  DDRD |= 1 << DDD1; // set LED pin PD1 to output
  TCNT2   = 0;
  TCCR2A |= (1 << WGM21); // Configure timer 2 for CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Start timer at Fcpu/64
  TIMSK2 |= (1 << OCIE2A); // Enable CTC interrupt
  OCR2A   = 65000; // Set CTC compare value with a prescaler of 64

  sei(); // Enable global interrupts

}
int a;

int timerCounter = 0;
int timerMultiplier = 20;
ISR(TIMER2_COMPA_vect)
{

}



void ISR1_Callback() {
  //
  if (start_timer) {

    if (callibration_done) {
      message = String(ang_euler.yaw) + "," + String(ang_euler.pitch) + "," + String(ang_euler.roll);
      message = message + "," + String(joystick.X_value) + "," + String(joystick.Y_value) + "," + String(joystick.button_value);
      message = message + "," + String(eu.roll);
      // Send data
      tmserial.println(message);
    }
    else if (millis() > 15000 && !callibration_done) {
      callibrate_yaw = ang_euler.yaw;
      callibrate_roll = ang_euler.roll;
      callibrate_pitch = ang_euler.pitch;
      Serial.println("Callibration");
      callibration_done = true;

    }

  }
}




void setup() {
  // setTimer2();
  // Joystick R3 button

  // Uncomment this code in order to use better MPU9250 code, but not working with two accelerometers
  // pinMode(pinJoyButton , INPUT_PULLUP);
  Wire.begin();
  // Serial communication
  Serial.begin(115200);
  Serial.println("Inicializando");
  // Set timer and attach interruption function
  Timer1.initialize(70000);
  Timer1.attachInterrupt( ISR1_Callback);
  // Bluetooth communication
  tmserial.begin(115200);
  // Giroscope set up
  gyroscope_setup();
  Accelerometer.initialize_I2C();
  // Start timer when setup is ready
  start_timer = true;


}

void loop() {
  // Read euler angles from gyroscope
  ang_euler = gyroscope_read();
  // Read joystick
  joystick = get_joystick(joystick, pinJoyX, pinJoyY, pinJoyButton);
  eu = Accelerometer.calculer_angles();


  // Print values
  Serial.print("Yaw: ");
  Serial.print(ang_euler.yaw);
  Serial.print(", Roll: ");
  Serial.print(ang_euler.roll);
  Serial.print(", Pitch: ");
  Serial.print(ang_euler.pitch);
//  Serial.print(", X: ");
//  Serial.print(joystick.X_value);
//  Serial.print(", Y: ");
//  Serial.print(joystick.Y_value);
//  Serial.print(", Button: ");
//  Serial.print(joystick.button_value);
  Serial.print(", Angle: ");
  Serial.println(eu.roll);

}





// Uncomment this code in order to use better MPU9250 code, but not working with two accelerometers
//struct eulerAngles gyroscope_read() {
//  // if programming failed, don't try to do anything
//  if (!dmpReady) return;
//
//  // wait for MPU interrupt or extra packet(s) available
//  while (!mpuInterrupt && fifoCount < packetSize) {
//    if (mpuInterrupt && fifoCount < packetSize) {
//      // try to get out of the infinite loop
//      fifoCount = mpu.getFIFOCount();
//    }
//
//  }
//
//  // reset interrupt flag and get INT_STATUS byte
//  mpuInterrupt = false;
//  mpuIntStatus = mpu.getIntStatus();
//
//  // get current FIFO count
//  fifoCount = mpu.getFIFOCount();
//  if (fifoCount < packetSize) {
//    //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
//    // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//  }
//  // check for overflow (this should never happen unless our code is too inefficient)
//  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
//    // reset so we can continue cleanly
//    mpu.resetFIFO();
//    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
//    Serial.println(F("FIFO overflow!"));
//
//  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
//
//    while (fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
//      mpu.getFIFOBytes(fifoBuffer, packetSize);
//      fifoCount -= packetSize;
//    }
//
//    // display Euler angles in degrees
//    mpu.dmpGetQuaternion(&q, fifoBuffer);
//    mpu.dmpGetGravity(&gravity, &q);
//    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//
//    ang_euler.yaw = ypr[0] * 180 / M_PI;
//    ang_euler.roll = ypr[1] * 180 / M_PI;
//    ang_euler.pitch = ypr[2] * 180 / M_PI;
//
//
//  }
//  return ang_euler;
//}
// Uncomment this code in order to use better MPU9250 code, but not working with two accelerometers

//void gyroscope_setup() {
//  // join I2C bus (I2Cdev library doesn't do this automatically)
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//  Wire.begin();
//  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
//#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//  Fastwire::setup(400, true);
//#endif
//  mpu.initialize();
//  pinMode(INTERRUPT_PIN, INPUT);
//
//  // verify connection
//  mpu.testConnection();
//
//
//  // load and configure the DMP
//
//  devStatus = mpu.dmpInitialize();
//
//  // supply your own gyro offsets here, scaled for min sensitivity
//  mpu.setXGyroOffset(220);
//  mpu.setYGyroOffset(76);
//  mpu.setZGyroOffset(-85);
//  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
//
//  // make sure it worked (returns 0 if so)
//  if (devStatus == 0) {
//    // Calibration Time: generate offsets and calibrate our MPU6050
//    mpu.CalibrateAccel(6);
//    mpu.CalibrateGyro(6);
//    // mpu.PrintActiveOffsets();
//    // turn on the DMP, now that it's ready
//    mpu.setDMPEnabled(true);
//
//    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
//    mpuIntStatus = mpu.getIntStatus();
//
//    // set our DMP Ready flag so the main loop() function knows it's okay to use it
//    dmpReady = true;
//
//    // get expected DMP packet size for later comparison
//    packetSize = mpu.dmpGetFIFOPacketSize();
//  } else {
//
//  }
//}

//void dmpDataReady() {
//  // Read gyroscope when there's and interruption in the INT pin of the gyroscope
//  mpuInterrupt = true;
//
//}
