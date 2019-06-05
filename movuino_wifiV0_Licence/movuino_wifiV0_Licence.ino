
#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#else

#include <WiFi.h>
#include <WiFiMulti.h>
/*#include <WiFiClient.h>
#include <WiFiServer.h>
#include <ETH.h>
#include <WiFiAP.h>
#include <WiFiGeneric.h>
#include <WiFiScan.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
*/
#include "SPIFFS.h"
#include <ESPmDNS.h>

#endif


#include <WiFiUdp.h>
//V0.3 added UDP
//0.31b testing write on memory until no battery or no space
#include "FS.h"
#include "Wire.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <Ticker.h>  //Ticker Library

// #include <i2c_adc_ads7828.h>

Adafruit_BME280 bme; // I2C

Ticker ticker1;


// device 0
// Address: A1=0, A0=0
// Command: SD=1, PD1=1, PD0=1
//ADS7828 device(0, SINGLE_ENDED | REFERENCE_ON | ADC_ON/*, 0x0F*/); //no mask to do find how it works
//ADS7828* adc = &device;
//ADS7828Channel* Analog0= adc->channel(0);
//ADS7828Channel* Analog1= adc->channel(1);
int opMode = 0;
long int timer0;
long int timer1;
long int startTimer;
long int sampleNb;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
MPU6050 accelgyro;
//Led and buttons
const int buttonPin = 13;
//variables for the debounce
int buttonState = 0;  // variable for reading the pushbutton status
long lastButtonTime = 0;
int debounceDelay = 400;
float freq = 10;  // max frequency: 333hz
float period = 100; // min period: 3ms

//wifi +UDP variables
#ifdef ESP8266
ESP8266WiFiMulti WiFiMulti;
#else
WiFiMulti WiFiMulti; // ESP8266 : ESP8266WiFiMulti WiFiMulti;
#endif
WiFiClient client;
const uint16_t port = 1000;
const char * host = "192.168.43.38"; // ip or dns
char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "S: 1 2 3 4 1254 1245 1234";       // a string to send back
unsigned int localPort = 2390;      // local port to listen on
WiFiUDP Udp;

File fw;
#ifdef ESP8266
void memory_info(){
    FSInfo fs_info;
    SPIFFS.info(fs_info);
    Serial.print("totalBytes :");
    Serial.println(fs_info.totalBytes);
    Serial.print("usedBytes :");
    Serial.println(fs_info.usedBytes);
}
void list_files(){
    Dir dir = SPIFFS.openDir("/data");
    while (dir.next()) {
       Serial.print(dir.fileName());
       File f = dir.openFile("r");
       Serial.println(f.size());
      }
}
#else
/*
void memory_info(){
    FSInfo fs_info;
    SPIFFS.info(fs_info);
    Serial.print("totalBytes :");
    Serial.println(fs_info.totalBytes);
    Serial.print("usedBytes :");
    Serial.println(fs_info.usedBytes);
} */
void list_files(){
    File dir = SPIFFS.open("/data"); // ESP8266 : Dir dir = SPIFFS.openDir("/data");
    for (int cnt = 0; true; ++cnt) { //ESP8266 : while (dir.next()) {
       Serial.print(dir.name()); // ESP8266 : Serial.print(dir.fileName());
       File f = dir.openNextFile("r"); // ESP8266 : File f = dir.openFile("r");
       if (!f)
       break;
       Serial.print(" ");
       Serial.println(f.size());
      }
}
#endif
void read_memory() {
  // nothing to do for now, this is just a simple test
  File f = SPIFFS.open("/data/1.txt", "r");
  // we could open the file
  while (f.available()) {
    //Lets read line by line from the file
    String line = f.readStringUntil('\n');
    Serial.println(line);
    delay(5);
   // yield();
  }
  f.close();
}

void Connect_Wifi() {
  //WiFiMulti.addAP("MotoG3", "z12345678");
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  //WiFi.begin("Ke20 iPhone", "z12345678");
  // WiFiMulti.addAP("MotoG3", "z12345678");
  delay(100);
  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.println("connecting ..");
    //WiFi.begin("MotoG3", "z12345678");
    WiFiMulti.addAP("AndroidAP", "ssxw9961");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(50);
  //WiFi.mode(WIFI_OFF);
}
/*
void Send_Data() {
  Udp.begin(localPort);
  char msg[30];
  //accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Udp.beginPacket(host, 2390);
  sprintf(msg, "S: 1 2 3 a ");
  Udp.write((uint8_t)msg);
  Udp.endPacket();
  // if(packetNumber<255) packetNumber ++;
  // else packetNumber=0;
  //sprintf(msg, "S: 1 2 3 a %d %d %d",ax,ay,az);
  //sprintf(msg, "S: %d 2 3 a %d %d %d",packetNumber,ax,ay,az);
  //sprintf(msg, "S: %d 2 3 a %d 123 123",packetNumber,packetNumber);
  //sprintf(msg, "S: %d 2 3 a 123 123 123",packetNumber);
  // Serial.println(ax);
  /* Udp.write(msg);
    Udp.endPacket();
    time0=micros();*/
// }  
void Send_Data_From_File() {
  Udp.begin(localPort);
  char msg[45];
  char msg2[40];
  // nothing to do for now, this is just a simple test
  File f = SPIFFS.open("/data/1.txt", "r");
  int lineNb = 0;
  String toto = "S: 12345678lklkmlkmlkkmlkmlkmlk";
  // we could open the file
  while (f.available()) {
    //Lets read line by line from the file
    String line = f.readStringUntil('\n');
    Udp.beginPacket(host, 2390);
    //attention ceette ligne doit etre absolumrnt après udp bediginpacket
    line.toCharArray(msg, line.length());
    //sprintf(msg2, "S: 1 2 3 a ");
    // toto.toCharArray(msg2, 20);
    Udp.print(msg);
    Udp.endPacket();
    Serial.print(lineNb);
    Serial.print("length :");
    Serial.print(line.length());
    Serial.print("msg:");
    Serial.println(msg);
    delay(10);
    lineNb++;
  }
  f.close();
  Serial.println("finish sending");
}

void onTick()
{
  
  if (opMode == 1) {
    /*
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    int sensorValue = analogRead(A0);
    Serial.print(timer0);
    Serial.print(" ");
    Serial.print(ax);
    Serial.print(" ");
    Serial.print(ay);
    Serial.print(" ");
    Serial.print(az);
    Serial.print(" ");
    Serial.print(gx);
    Serial.print(" ");
    Serial.print(gy);
    Serial.print(" ");
    Serial.print(gz);
    Serial.print(" ");
    Serial.println(sensorValue);
    //delay(2);
    */

    //Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.print(" *C");
    Serial.print(" \t");

    //Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.print(" hPa");
    Serial.print(" \t");

    //Serial.print(" mesure n° ");
    Serial.println(sampleNb);

      sampleNb ++;  

  }
  //print in file
  else if (opMode == 2) {

    /*
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    int sensorValue = analogRead(A0);
    */
    //fw.println("toto");
   // fw.print("S: ");
       //stop recording if <3.3v or more than 1 hour
       
   // if(sensorValue<600 /*|| sampleNb>9000*/) {
   /*
       fw.close();
       opMode=0;
       Serial.print("Stop recording");
       digitalWrite(2, HIGH);
      }
      else{
    fw.print(sampleNb);
    fw.print(" ");
  //  fw.print(ax);
  //  fw.print(" ");
  //  fw.print(ay);
  //  fw.print(" ");
  //  fw.print(az);
  //  fw.print(" ");
  //  fw.print(gx);
  //  fw.print(" ");
  //  fw.print(gy);
  //  fw.print(" ");
  //  fw.print(gz);
  //  fw.print(" ");
    fw.println(sensorValue);
    sampleNb++;
      }
      */
    fw.print(bme.readTemperature());
    fw.print(" *C");
    fw.print(" \t");

    //Serial.print("Pressure = ");
    fw.print(bme.readPressure() / 100.0F);
    fw.print(" hPa");
    fw.print(" \t");

    //Serial.print(" mesure n° ");
    fw.println(sampleNb);

    sampleNb ++;  
    

   
  }
  else if(opMode == 3){
      sampleNb++;

      //get data
      //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      //int sensorValue = analogRead(A0);
      //send udp data
      Udp.begin(localPort);
      char msg[30];
      Udp.beginPacket(host, 2390);
      // sprintf(msg, "S: %d 2 3 a %d",ax,ay);
      sprintf(msg, "%d %d %d",bme.readTemperature()," *C"," \t",bme.readPressure() / 100.0F, " hPa"," \t", sampleNb);
      //while (replyPacket[i] != 0) Udp.write((uint8_t)msg[i++]);
      Udp.print(msg);
      Udp.endPacket();   
      sampleNb ++;  
      //Serial.print("Temperature = ");
      Serial.print(bme.readTemperature());
      Serial.print(" *C");
      Serial.print(" \t");

      //Serial.print("Pressure = ");
      Serial.print(bme.readPressure() / 100.0F);
      Serial.print(" hPa");
      Serial.print(" \t");
    
    //Serial.print(" mesure n° ");
    Serial.println(sampleNb);
      
    /*  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    int sensorValue = analogRead(A0);*/
  }

}

void setup() {


    bool status;
    status = bme.begin(0x76);  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }  

  
  Serial.begin(115200);
  Wire.begin();
  // enable I2C communication
//  ADS7828::begin();
  // adjust scaling on an individual channel basis 12 bits = 4096
//  Analog1->minScale = 0;
//  Analog1->maxScale = 4095;
  // We start by connecting to a WiFi network
  WiFiMulti.addAP("MotoG3", "z12345678");
  //configure adc foor battery level monitoring
  pinMode(A0, INPUT);
  delay(2000);
  //blue built-in led
  pinMode(2, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
  // always use this to "mount" the filesystem
  bool result = SPIFFS.begin();
  //turn off led
  digitalWrite(2, HIGH);
  Serial.println("SPIFFS opened: " + result);
  // this opens the file "f.txt" in read-mode
  //  File f = SPIFFS.open("/f.txt", "r");
  //if file does not exist
  /*  if (!f) {
      Serial.println("File doesn't exist yet. Creating it");
      // open the file in write mode
      File f = SPIFFS.open("/f.txt", "w");
      if (!f) {
        Serial.println("file creation failed");
      }
      // now write two lines in key/value style with  end-of-line characters
      Serial.println("Writing in file");
      f.println("ssid=abc");
      f.println("password=123455secret");
    }
    //if file exist
    else {
      // we could open the file
      while(f.available()) {
        //Lets read line by line from the file
        String line = f.readStringUntil('n');
        Serial.println(line);
      }
    }
    f.close();*/
  //init MPU
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

ticker1.attach_ms(period, onTick);  // activate the ticker, to do a measurement every period

Serial.setTimeout(1); // serial timeout is 1ms to reduce the delay after sending  a command
}

void loop() {
  //Read button
  buttonState = digitalRead(buttonPin);
  if (!buttonState) {
    if (millis() - lastButtonTime > debounceDelay) {
      lastButtonTime = millis();
      //Serial.println("Recording...");
      //if we are recording stop recording
      if (opMode == 2) {
        //TODO fonction start_recording & stop_recording
        fw.close();
        Serial.println("Stop recording");
        digitalWrite(2, HIGH);
        opMode = 0;
      }
      else {
        Serial.println("Recording");
        fw = SPIFFS.open("/data/1.txt", "w+");
        opMode = 2;
        digitalWrite(2, LOW);
      }
    }
  }
  timer0 = millis();
  if (Serial.available() > 0) {
    String inByte = Serial.readStringUntil(13);
    //Serial.println(inByte);
    //Ping
    Serial.println(inByte);
    if (inByte.substring(0,1) == "?") {
     // Serial.println("?");
    }
    //create dir + files
    else if (inByte.substring(0,1) == "c") {
      Serial.println("Create file");

      File f = SPIFFS.open("/data/1.txt", "r");
      if (!f) {
        Serial.println("File doesn't exist yet. Creating it");
        File f = SPIFFS.open("/data/1.txt", "w");
        if (!f) {
          Serial.println("file creation failed");
          //put error code here
        }
        // now write two lines in key/value style with  end-of-line characters
        Serial.println("Writing in file");
        f.println("ssid=abc");
        f.println("password=123455secret");
        f.close();
      }
      else Serial.println("File exist!");
    }
    //dir
    else if (inByte.substring(0,1) == "d") {
      Serial.println("Listing dir :");
      digitalWrite(2, LOW);
      list_files();
      Serial.println("End of listing");
      digitalWrite(2, HIGH);
    }
    //read the data
    else if (inByte.substring(0,1) == "p") {
      Serial.println("Read");
      read_memory();
      Serial.println("End of Read");
    }
    //recording mode
    else if (inByte.substring(0,1) == "r") {
      if (opMode == 0) {
      Serial.println("Recording");
      fw = SPIFFS.open("/data/1.txt", "w+");
      startTimer=millis(); 
      opMode = 2;
      }
      else {
        Serial.println("stop the live before recording");
      }
      //digitalWrite(2, LOW);
    }
    //stop recording mode
    else if (inByte.substring(0,1) == "R") {
      if (opMode == 2) {      
      fw.close();
      //reset sample nb
      sampleNb=0;
      Serial.println("Stop recording");
      digitalWrite(2, HIGH);
      opMode = 0;
      }
      else { 
        Serial.println("you didn't start the recording");
      }
      //read_memory();
    }
    //delete file
    else if (inByte.substring(0,1) == "P") {
      Serial.println("Deleting file");
      SPIFFS.remove("/data/1.txt");
      Serial.println("deletion");
      opMode = 0;
      //read_memory();
    }
    //live mode
    else if (inByte.substring(0,1) == "l") {
      if (opMode == 0) {
      Serial.println("Start live");
      opMode = 1; //old 3
      }
      else {
        Serial.println("stop recording before beginning the live");
      }
    }
    //quit live mode
    else if (inByte.substring(0,1) == "L") {
      if (opMode == 1) {
      opMode = 0;
      sampleNb = 0;
      }
      else {
        Serial.println("you didn't start the live");
      }
    }
    //file info
    else if(inByte.substring(0,1)=="i"){
      Serial.println("Files info : ");
      list_files();
//      memory_info();
    }
    // determines the frequency of the measurements
    else if(inByte.substring(0,1)=="f"){
      freq = inByte.substring(2).toFloat();
      period = 1000/freq;
      Serial.println(freq);   
      Serial.println(period);  
      ticker1.attach_ms(period, onTick);  
      
    }
    // prints the frequency
    else if(inByte.substring(0,1)=="F"){
      Serial.print("f = ");
      Serial.print(freq);
      Serial.println(" Hz");
    }
        //sending data via udp
    else if (inByte.substring(0,1) == "s") {
      Serial.println("Connecting to Wifi");
      Connect_Wifi();
     // Serial.println("Sending data ");
     // Send_Data();
     // Send_Data_From_File();
    }
    else if (inByte.substring(0,1) == "S") {
      Serial.println("Turning off wifi");
      WiFi.mode(WIFI_OFF);
      WiFi.disconnect();
    }
        //scan networks
    else if (inByte.substring(0,1) == "x") {
      Serial.println("Scanning networks");
      int n = WiFi.scanNetworks();
      Serial.println("scan done");
      if (n == 0)
        Serial.println("no networks found");
      else
      {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i)
        {
          // Print SSID and RSSI for each network found
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print(WiFi.SSID(i));
          Serial.print(" (");
          Serial.print(WiFi.RSSI(i));
          Serial.print(")");
          #ifdef ESP8266
          Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
          #else
          Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*"); // ESP8266: Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
          #endif
          
          delay(10); 
        }
      }
      Serial.println("");
      // Wait a bit before scanning again
      delay(5000);
    }
    //send data live via udp 
    else if(inByte.substring(0,1)=="u"){
       Serial.println("Start UDP");
       Connect_Wifi(); 
       startTimer=millis(); 
       opMode = 3;
    }
    else if(inByte.substring(0,1)=="U"){
      Serial.println("Stop UDP");
      opMode = 0;
      sampleNb=0;
    }
  }
}
