# Robot-Teleoperation-Unity

Teleoperation of three degrees of freedom robot using Arduino, PCA9685, MPU9250 and Unity. The robot is teleoperated using the movement
of the operator's arm measured using a MPU9250 and a MPU6500.Then these values are sent to an Unity interface using a HC05 Bluetooth module.
The Unity interface simulate the robot and gives command to the real robot using WiFi. The robot receives the data through an ESP8266 and the
motors(Futaba3003) are moved using a PCA9685. (Full report available in Spanish)
