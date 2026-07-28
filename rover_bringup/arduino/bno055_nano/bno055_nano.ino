/*
  BNO055 IMU Serial Publisher for Arduino Nano
  --------------------------------------------
  Connections:
  - BNO055 VCC -> Arduino 5V or 3.3V (depending on board)
  - BNO055 GND -> Arduino GND
  - BNO055 SDA -> Arduino A4
  - BNO055 SCL -> Arduino A5

  Uses OPERATION_MODE_IMUPLUS: Accelerometer + Gyroscope fusion.
  NO MAGNETOMETER CALIBRATION REQUIRED! Fast, drift-free turning.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Check I2C address: 0x28 (default) or 0x29 (if ADR pin is HIGH)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void) {
  Serial.begin(115200);
  
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    // Try alternate address 0x29 if 0x28 fails
    bno = Adafruit_BNO055(55, 0x29);
    if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
      Serial.println("ERR: BNO055 not detected on I2C (0x28/0x29)");
      while (1) {
        delay(500);
      }
    }
  }

  bno.setExtCrystalUse(true);
  Serial.println("INFO: BNO055 Initialized in IMUPLUS mode");
}

void loop(void) {
  imu::Quaternion quat = bno.getQuat();
  
  // Format: QUAT:w,x,y,z
  Serial.print("QUAT:");
  Serial.print(quat.w(), 4);
  Serial.print(",");
  Serial.print(quat.x(), 4);
  Serial.print(",");
  Serial.print(quat.y(), 4);
  Serial.print(",");
  Serial.println(quat.z(), 4);

  delay(20); // 50 Hz output rate
}
