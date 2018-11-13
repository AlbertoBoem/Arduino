//ESP8266
//Adafruit LSM9DSO 9dof IMU
//Transmit data through OSC protocol

//NodeMCU 1.0 (ESP-12E Module) 80mhz - SLAB_USBtoUART
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <OSCMessage.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

#include <Adafruit_Simple_AHRS.h>

//Network
char ssid[] = "";          //network SSID (name)
char pass[] = "";          // network password


WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP

//Out to MAIN PC
const IPAddress outIp(255,255,255,255);            // remote IP of your computer

//Network setting for each device
IPAddress ip(, , , ); //ip of the device 
IPAddress gateway(, , , );
IPAddress subnet(255, 255, 255, 0);
IPAddress DNS(, , , );

//OSC Port
const unsigned int outPort = 7002;          // remote port to receive OSC
const unsigned int localPort = 8889;        // local port to listen for OSC packets (actually not used for sending)

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());


void configureSensor(void)
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G); // DEFAULT
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G); // test 2018/05/10 --> To measure the acceleration of gravity for use as a tilt sensor, an output range of ±1.5 g is sufficient. For use as an impact sensor, one of the most common musical applications, ±5 g or more is desired.
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup() {
    Serial.begin(115200);

    // Connect to WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.config(ip, gateway, subnet, DNS);
    delay(100);
    WiFi.begin(ssid, pass);

    if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Display some basic information on this sensor */
  //displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");

    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Starting UDP");
    Udp.begin(localPort);
    Serial.print("Local port: ");
#ifdef ESP32
    Serial.println(localPort);
#else
    Serial.println(Udp.localPort());
#endif

}

void loop() {

sensors_event_t accel, mag, gyro, temp;

lsm.getEvent(&accel, &mag, &gyro, &temp); 

sensors_vec_t   orientation;

    if (ahrs.getOrientation(&orientation)){
    
    OSCMessage msg("/deviceLeft");
    msg.add(accel.acceleration.x);
    msg.add(accel.acceleration.y);
    msg.add(accel.acceleration.z);
    msg.add(gyro.gyro.x);
    msg.add(gyro.gyro.y);
    msg.add(gyro.gyro.z);
    
    msg.add(orientation.roll);
    msg.add(orientation.pitch);
    msg.add(orientation.heading);
    
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    
    delay(10);
    }
/*
    Serial.print(accel.acceleration.x);
    Serial.println(" ");
    Serial.println(accel.acceleration.y);
    Serial.println(" ");
    Serial.println(accel.acceleration.z);
 */
}
