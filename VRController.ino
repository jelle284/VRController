#include "I2Cdev.h"
#include "MPU9250_9Axis_MotionApps41.h"
#include "Wire.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//                                        Data packet
// ======================================================================================
struct DataPacket_t {
  float quat[4], acc[3], gyro[3];
  bool TriggerBtn;
} DataPacket;

//                                        Button State
// ======================================================================================
#define TRIGGER_PIN 15
bool TriggerButton;

//                                        MPU variables
// ======================================================================================

MPU9250 mpu;

#define INTERRUPT_PIN 3

const float ACC_SCALING = 0.00119750977;
const float GYRO_SCALING = 0.00002961383;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;       // [w, x, y, z]   quaternion container
VectorInt16 Acc;    // [x, y, z]      Linear acceleration
int16_t gyro[3];    //                Angular vel

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//                                      ESP
// https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/udp-examples.html
// ======================================================================================

const char* ssid = "123";
const char* password = "123";

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[256];  // buffer for incoming packets

//                                    LED
// ======================================================================================
#define RED_PIN 13
#define GREEN_PIN 12
#define BLUE_PIN 14

void setColor(int r, int g, int b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}

//                                   SETUP
// ======================================================================================
void setup()
{
  // Set up LED pins. Use D5, D6, D7.
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);

  // Set up button.
  pinMode(TRIGGER_PIN, INPUT);

  // Initiate serial debugging
  Serial.begin(9600);
  Serial.printf("Connecting to %s ", ssid);

  // Initiate WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  // Start up MPU
  Serial.printf("i2c\n");
  Wire.begin(4, 5);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.printf("Initializing MPU...\n");
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    Serial.printf("Starting dmp...\n");
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.printf("MPU not connected\n");
    dmpReady = false;
  }
  Serial.printf("Running...\n");
  Serial.end();
}

//                             MAIN LOOP
// ======================================================================================
void loop() {
  // Reset buffer
  memset(incomingPacket, '\0', sizeof(incomingPacket));
  memset((char*)(&DataPacket), '\0', sizeof(DataPacket));

  // Read MPU data
  if (dmpReady) {
    while (!mpuInterrupt && fifoCount < packetSize) {
      // waiting for MPU data
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {

      while (fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
      }
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      // Read quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);

      // Read acceleration
      mpu.dmpGetAccel(&Acc, fifoBuffer);

      // read gyro
      mpu.dmpGetGyro(gyro, fifoBuffer);

      mpu.resetFIFO();
    }
  }
  
  // Recieve instructions from PC
  if (Udp.parsePacket()) {
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
  }

  // Send ID
  if (strcmp(incomingPacket, "id") == 0) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.print("Right Hand Controller");
    Udp.endPacket();
  }

  // send full sample [q, a, w]
  if (strcmp(incomingPacket, "sample") == 0) {
    DataPacket.quat[0] = q.w;
    DataPacket.quat[1] = q.x;
    DataPacket.quat[2] = q.y;
    DataPacket.quat[3] = q.z;

    DataPacket.acc[0] = ACC_SCALING * Acc.x;
    DataPacket.acc[1] = ACC_SCALING * Acc.y;
    DataPacket.acc[2] = ACC_SCALING * Acc.z;

    DataPacket.gyro[0] = GYRO_SCALING * gyro[0];
    DataPacket.gyro[1] = GYRO_SCALING * gyro[1];
    DataPacket.gyro[2] = GYRO_SCALING * gyro[2];

    DataPacket.TriggerBtn = TriggerButton;
    TriggerButton = false;

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write((char*)(&DataPacket), sizeof(DataPacket));
    Udp.endPacket();
  }

  // LED commands
  if (strcmp(incomingPacket, "off") == 0) {
    setColor(0, 0, 0);
  }
  if (strcmp(incomingPacket, "red") == 0) {
    setColor(255, 0, 0);
  }
  if (strcmp(incomingPacket, "green") == 0) {
    setColor(0, 255, 0);
  }
  if (strcmp(incomingPacket, "blue") == 0) {
    setColor(0, 0, 255);
  }
  // Read button state
  if (!TriggerButton) TriggerButton = digitalRead(TRIGGER_PIN);
}


