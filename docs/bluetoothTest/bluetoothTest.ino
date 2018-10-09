 #include <SoftwareSerial.h>// import the serial library

SoftwareSerial Genotronex(10, 11); // RX, TX
int BluetoothData; // the data given from Computer

void setup() {
 // put your setup code here, to run once:
Genotronex.begin(9600);

 }

void loop() {
  String A = "Hola";
  BluetoothData = 105;
  Genotronex.print(A);
//
//    if (Genotronex.available())
//    {
//        BluetoothData=Genotronex.read();
//        Genotronex.write(BluetoothData);
//    }
    delay(100);

}
