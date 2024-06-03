#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Servo.h>


Adafruit_LIS3DH lis = Adafruit_LIS3DH();
Servo servoMotor;


void initializeSerial() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
}


void initializeAccelerometer() {
  if (!lis.begin(0x18)) {
    Serial.println("Couldn't start accelerometer");
    while (1);
  }
  Serial.println("LIS3DH accelerometer found!");
  lis.setRange(LIS3DH_RANGE_4_G);
}


void initializeServo() {
  servoMotor.attach(9);
  Serial.println("Servo motor initialized on pin 9");
}


float readZAxisAcceleration() {
  lis.read();
  float accel_z = lis.readFloat(Z_AXIS);
  Serial.print("Z-axis acceleration: ");
  Serial.println(accel_z, 2);
  return accel_z;
}


void controlServoBasedOnAcceleration(float accel_z) {
  if (abs(accel_z) > 2.0) {
    Serial.println("Rapid movement detected");
    servoMotor.write(0);
  } else {
    Serial.println("Little movement detected");
    servoMotor.write(180);
  }
}


void setup() {
  initializeSerial();
  initializeAccelerometer();
  initializeServo();
}


void loop() {
  float accel_z = readZAxisAcceleration();
  controlServoBasedOnAcceleration(accel_z);
  delay(1000);
}
