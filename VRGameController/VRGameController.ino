#define INVERT_X   //invert pitch
#define INVERT_Y   //invert roll

//#define DEBUG          //uncomment this for testing, 
                         //comment it before making joystick


#include "UnoJoy.h"
#include <Wire.h>

#define XSensitivity 3.0
#define YSensitivity 3.0


/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double tempRaw;
double dt;

double compAngleX, compAngleY; // Calculated angle using a complementary filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


void setup() {

#ifdef DEBUG
  Serial.begin(38400);
  Serial.println("Ready");
  Serial.print("compAngleX");  Serial.print('\t');
  Serial.print("compAngleY");  Serial.print('\t');
  Serial.print("leftStickX");  Serial.print('\t');
  Serial.print("leftStickY");  Serial.print('\t');
  Serial.print("rightStickX");  Serial.print('\t');
  Serial.print("rightStickY");  Serial.print('\t');
  Serial.println();
  delay(4000);
#endif

  InitializeGPIO();
  InitializeMpu();
  delay(100); // Wait for sensor to stabilize

  UpdateReadings();     //Updates accel/gyro readings
  SetInitialValues();   //Set inital roll and pitch values

  setupUnoJoy();
  timer = micros();
}

void loop() {

  UpdateReadings();
  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  ComputeAngles();

  dataForController_t controllerData = getControllerData(compAngleX * XSensitivity, compAngleY * YSensitivity);

#ifdef DEBUG
  Serial.print(compAngleX);  Serial.print('\t');
  Serial.print(compAngleY);  Serial.print('\t');
  Serial.print(controllerData.leftStickX);  Serial.print('\t');
  Serial.print(controllerData.leftStickY);  Serial.print('\t');
  Serial.print(controllerData.rightStickX);  Serial.print('\t');
  Serial.print(controllerData.rightStickY);  Serial.print('\t');
  Serial.println();
#else
  setControllerData(controllerData);
#endif

  delay(2);
}

void InitializeGPIO()
{
  for (int i = 2; i <= 5; i++)
  {
    pinMode(i, INPUT);
    digitalWrite(i, HIGH);
  }
}


void InitializeMpu()
{
  Wire.begin();
  Wire.setClock(400000UL);

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68)
  { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
}


void UpdateReadings()
{
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

}


void SetInitialValues()
{
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  compAngleX = roll;
  compAngleY = pitch;
}


void ComputeAngles()
{
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  compAngleX = 0.98 * (compAngleX + gyroXrate * dt) + 0.02 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.98 * (compAngleY + gyroYrate * dt) + 0.02 * pitch;
}


dataForController_t getControllerData(double _roll, double _pitch) {

#ifdef INVERT_X
  _pitch = -_pitch;
#endif
#ifdef INVERT_Y
  _roll = -_roll;
#endif

  //Restrict roll and pitch to +-90
  if (_roll > 90 || _roll < -90)
    _roll = 90 * _roll / abs(_roll);

  if (_pitch > 90 || _pitch < -90)
    _pitch = 90 * _pitch / abs(_pitch);


  //Get new instance with default values of buttons and sticks
  dataForController_t controllerData = getBlankDataForController();

  controllerData.leftStickX = (int)(_pitch * 1.422 + 128);
  controllerData.leftStickY =  (int)(_roll * 1.422 + 128);

  //You can use these and many other buttons also
  controllerData.triangleOn = !digitalRead(2);
  controllerData.circleOn = !digitalRead(3);
  controllerData.squareOn = !digitalRead(4);
  controllerData.crossOn = !digitalRead(5);

  return controllerData;
}


