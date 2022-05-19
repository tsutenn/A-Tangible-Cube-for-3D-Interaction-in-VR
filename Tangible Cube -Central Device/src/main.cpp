/*
 * This is the central device
 * BT Address: 10082c1f3d9d
 * Service UUID: 00001800-0000-1000-8000-00805f9b34fb
 */

#include <Arduino.h>
// #include <ArduinoBLE.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps612.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

/*
 * Serial and Buletooth LE
 */

const char* deviceServiceUuid = "00001800-0000-1000-8000-00805f9b34fb";
const char* deviceServiceCharacteristicUuid = "00001800-0000-1000-8000-00805f9b34fb";

void setupNano(){
  Serial.begin(9600);
  while(!Serial);

  // if (!BLE.begin()) {
  //   Serial.println("* Starting Bluetooth® Low Energy module failed!");
  //   while (1);
  // }

  // BLE.setLocalName("Nano 33 BLE (Central)"); 
  // BLE.advertise();

  // Serial.println("Arduino Nano 33 BLE Sense (Central Device)");
  // Serial.println(" ");
}

/*
 * quaternion from MPU6050
 */

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

MPU6050 mpu;

void setupIMU(){
  Wire.begin();
  Wire.setClock(100000);

  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    //turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void getQuaternion(){
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

/*
 * pressure
 */

#define PRESS_MIN	0
#define PRESS_MAX	99
#define VOLTAGE_MIN 0
#define VOLTAGE_MAX 4300
int sensorPin1 = A1;
int sensorPin2 = A2;
int sensorPin3 = A3;

int p[] = {0, 0, 0};

void getPressure()
{
	int VOLTAGE_AO = 0;
	int value[3] = {
    analogRead(sensorPin1),
    analogRead(sensorPin2),
    analogRead(sensorPin3)
    };

  for(int i = 0; i < 3; i++){
    VOLTAGE_AO = map(value[i], 0, 1023, 0, 5000);

    if(VOLTAGE_AO < VOLTAGE_MIN)
    {
      p[i] = 0;
    }
    else if(VOLTAGE_AO > VOLTAGE_MAX)
    {
      p[i] = PRESS_MAX;
    }
    else
    {
      p[i] = map(VOLTAGE_AO, VOLTAGE_MIN, VOLTAGE_MAX, PRESS_MIN, PRESS_MAX);
    }
  }
	
}

/*
 * print data
 */

void printAll(){
  Serial.print("qw=");
  Serial.print(q.w);
  Serial.print(",qx=");
  Serial.print(q.x);
  Serial.print(",qy=");
  Serial.print(q.y);
  Serial.print(",qz=");
  Serial.print(q.z);
  Serial.print(",p0=");
  Serial.print(p[0]);
  Serial.print(",p1=");
  Serial.print(p[1]);
  Serial.print(",p2=");
  Serial.print(p[2]);
  Serial.println("");
}

char* str = "          ";

void writeAll(){
  sprintf(str, "          ");

  char qws = (q.w >= 0) ? '+' : '-';
  char qxs = (q.x >= 0) ? '+' : '-';
  char qys = (q.y >= 0) ? '+' : '-';
  char qzs = (q.z >= 0) ? '+' : '-';

  int qwi = q.w;
  int qwf = q.w*100 - qwi*100;
  int qxi = q.x;
  int qxf = q.x*100 - qxi*100;
  int qyi = q.y;
  int qyf = q.y*100 - qyi*100;
  int qzi = q.z;
  int qzf = q.z*100 - qzi*100;

  qwf = abs(qwf);
  qxf = abs(qxf);
  qyf = abs(qyf);
  qzf = abs(qzf);

  sprintf(str, "qw=%c%d.%02d,", qws, qwi, qwf);
  Serial.write(str);
  sprintf(str, "qx=%c%d.%02d,", qxs, qxi, qxf);
  Serial.write(str);
  sprintf(str, "qy=%c%d.%02d,", qys, qyi, qyf);
  Serial.write(str);
  sprintf(str, "qz=%c%d.%02d,", qzs, qzi, qzf);
  Serial.write(str);
  sprintf(str, "p0=%d,", p[0]);
  Serial.write(str);
  sprintf(str, "p1=%d,", p[1]);
  Serial.write(str);
  sprintf(str, "p2=%d\n", p[2]);
  Serial.write(str);
}

void writeQuaternion(){
  sprintf(str, "          ");

  char qws = (q.w >= 0) ? '+' : '-';
  char qxs = (q.x >= 0) ? '+' : '-';
  char qys = (q.y >= 0) ? '+' : '-';
  char qzs = (q.z >= 0) ? '+' : '-';

  int qwi = q.w;
  int qwf = q.w*100 - qwi*100;
  int qxi = q.x;
  int qxf = q.x*100 - qxi*100;
  int qyi = q.y;
  int qyf = q.y*100 - qyi*100;
  int qzi = q.z;
  int qzf = q.z*100 - qzi*100;

  qwf = abs(qwf);
  qxf = abs(qxf);
  qyf = abs(qyf);
  qzf = abs(qzf);

  sprintf(str, "qw=%c%d.%02d,", qws, qwi, qwf);
  Serial.write(str);
  sprintf(str, "qx=%c%d.%02d,", qxs, qxi, qxf);
  Serial.write(str);
  sprintf(str, "qy=%c%d.%02d,", qys, qyi, qyf);
  Serial.write(str);
  sprintf(str, "qz=%c%d.%02d\n", qzs, qzi, qzf);
  Serial.write(str);
}

void writePressure(){
  sprintf(str, "          ");

  sprintf(str, "p0=%02d,", p[0]);
  Serial.write(str);
  sprintf(str, "p1=%02d,", p[1]);
  Serial.write(str);
  sprintf(str, "p2=%02d\n", p[2]);
  Serial.write(str);
}

void writeWX(){
  sprintf(str, "          ");

  char qws = (q.w >= 0) ? '+' : '-';
  int qwi = q.w;
  int qwf = q.w*100 - qwi*100;
  qwf = abs(qwf);
  

  char qxs = (q.x >= 0) ? '+' : '-';
  int qxi = q.x;
  int qxf = q.x*100 - qxi*100;
  qxf = abs(qxf);

  sprintf(str, "qw=%c%d.%02d,", qws, qwi, qwf);
  Serial.write(str);
  sprintf(str, "qx=%c%d.%02d\n", qxs, qxi, qxf);
  Serial.write(str);
}

void writeYZ(){
  sprintf(str, "          ");

  char qys = (q.y >= 0) ? '+' : '-';
  int qyi = q.y;
  int qyf = q.y*100 - qyi*100;
  qyf = abs(qyf);

  char qzs = (q.z >= 0) ? '+' : '-';
  int qzi = q.z;
  int qzf = q.z*100 - qzi*100;
  qzf = abs(qzf);
  sprintf(str, "qy=%c%d.%02d,", qys, qyi, qyf);
  Serial.write(str);
  sprintf(str, "qz=%c%d.%02d\n", qzs, qzi, qzf);
  Serial.write(str);
}

/*
 * setup and loop
 */

void setup() {
  // put your setup code here, to run once:
  setupNano();
  setupIMU();
}

void loop() {
  // put your main code here, to run repeatedly:
  int r = Serial.read();

  if(r == 'Q') {
    getQuaternion();
    writeWX();
  }

  if(r == 'q'){
    getQuaternion();
    writeYZ();
  }

  if(r == 'P') {
    getPressure();
    writePressure();
  }

  if(r == 'A') {
    getQuaternion();
    getPressure();
    printAll();
  }
}
