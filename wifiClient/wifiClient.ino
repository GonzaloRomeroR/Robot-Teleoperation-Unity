// Program thar receives data from 3DOF robot simulation in order to
// move a robot with the same amount of degrees of freedom using an ESP8266
// Wifi module(Nodemcu 1.0) and a PCA9685 motor driver.

// ESP8266 Library
#include <ESP8266WiFi.h>

// I2C Communication
#include <Wire.h>

// Library to control PCA9685
#include <Adafruit_PWMServoDriver.h>

// Timers
// #include <Ticker.h>
//Ticker my_timer;

// Set I2C address
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

// Set max and min position of the Servos (Futaba S3003)
unsigned int pos0 = 115; // Pulse width for position 0°
unsigned int pos180 = 445; // Pulse width for position 180°

// Credentials to connect to Wifi
char* ssid = "Telefonica";
const char* password =  "20582589";

// Credentials to connect to server
const uint16_t port = 8080;
const char * host = "192.168.1.34";

// Zeros:
// Coxa: 115: ángulos inferiores hacía izquierda.
// Fémur: 122: ángulos inferiores hacía arriba.
// Tibia: 150: ángulos superiores hacía arriba.

// Joint variables
int art_coxa = 115;
int art_femur = 122;
int art_tibia = 150;

// Joint zeros
int zero_coxa = 115;
int zero_femur = 122;
int zero_tibia = 150;

// Auxiliary variable
int aux_coxa = 115;
int aux_femur = 122;
int aux_tibia = 150;

// Joint direction
int dir_coxa = 1;
int dir_femur = -1;
int dir_tibia = -1;

// Joystick variables
int X = 0;
int Y = 0;
int button = 0;

// Command interpreter variables
int counter = 0;
char command[30];


// For testing
//int aux = 0;
//void moveRobot(){
//  if (aux == 0){
//    setServo(15,100);
//    aux = 1;
//  }
//  else {
//    setServo(15,30);
//    aux = 0;
//  }
//}

void setup() {
  // For testing
  //my_timer.attach(0.5, moveRobot);

  // Set baud rate
  Serial.begin(115200);

  // Wifi connection
  Serial.println("Connecting to Wifi ");
  WiFi.begin("Telefonica", "20582589");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }
  Serial.print("WiFi connected with IP:");
  Serial.println(WiFi.localIP());

  // Initialize servo driver
  servos.begin();
  servos.setPWMFreq(50);
}


// Set PWM frequency in order to move the servos
void setServo(uint8_t n_servo, int angulo) {
  int duty;
  duty = map(angulo, 0, 180, pos0, pos180);
  servos.setPWM(n_servo, 0, duty);
}



// Command interpreter
void command_interpreter(char command[], int size) {
  int value;
  if (size > 2) {
    value = atoi(&command[2]);
  }
  switch (command[1]) {
    case 'C':
      aux_coxa = value * dir_coxa + zero_coxa;
      if (aux_coxa > 180) {
        art_coxa = 180;
      }
      else {
        if (aux_coxa < 0) {
          art_coxa = 0;
        }
        else {
          art_coxa = aux_coxa;
        }
      }

      break;
    case 'F':
      aux_femur = value * dir_femur + zero_femur;
      if (aux_femur > 180) {
        art_femur = 180;
      }
      else {
        if (aux_femur < 0) {
          art_femur = 0;
        }
        else {
          art_femur = aux_femur;
        }
      }
      break;
    case 'T':
      aux_tibia = value * dir_tibia + zero_tibia;
      if (aux_tibia > 180) {
        art_tibia = 180;
      }
      else {
        if (aux_tibia < 0) {
          art_tibia = 0;
        }
        else {
          art_tibia = aux_tibia;
        }
      }
      break;
    case 'X':
      X = value;
      break;
    case 'Y':
      Y = value;
      break;
    case 'B':
      button = value;
      break;
    default:
      break;
  }
}


// Get command from the application
void get_command(char character) {
  switch (character)
  {
    // Beginning of the command
    case ':':
      counter = 0;
      command[counter++] = character;
      break;
    // Command finished
    case '=':
      command[counter] = 0;
      command_interpreter(command, counter);
      break;
    default:
      command[counter++] = character;
      break;
  }
}


// Print values read
void printJointVariables() {
  Serial.print("Coxa: ");
  Serial.print(art_coxa);
  Serial.print(";  Femur: ");
  Serial.print(art_femur);
  Serial.print(";  Tibia: ");
  Serial.println(art_tibia);
//  Serial.print(";  X: ");
//  Serial.print(X);
//  Serial.print(";  Y: ");
//  Serial.print(Y);
//  Serial.print(";  Button: ");
//  Serial.println(button);
}



void loop() {

  // Connect to Server
  WiFiClient client;
  if (client.connect(host, 8080))
  {
    Serial.println("connected");

    // Read values from server
    while (client.connected() || client.available())
    {
      if (client.available())
      {
        // Read line
        String line = client.readStringUntil('\n');

        for (int i = 0; i < 7; i++) {
          get_command(line[i]);
        }
        
        setServo(15, art_coxa);
        setServo(14, art_femur);
        setServo(13, art_tibia);
        
        printJointVariables();

      }
    }
    // Client stop
    client.stop();
    Serial.println("\n[Disconnected]");
  }
  else
  {
    Serial.println("connection failed!");
    client.stop();
  }
  delay(1000);
}
