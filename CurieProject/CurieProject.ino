#include "CurieIMU.h"
#include "CurieBLE.h"
#include <stdio.h>      /* puts, printf */

int aix, aiy, aiz;    // raw accelerometer values
int gix, giy, giz;    // raw gyro values
bool flag = false;

BLEPeripheral blePeripheral;
BLEService sensorService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLEUnsignedLongCharacteristic stepCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead);  // stepcount
BLEFloatCharacteristic axCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify); // acc.in x in g
BLEFloatCharacteristic ayCharacteristic("19B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify); // acc.in y in g
BLEFloatCharacteristic azCharacteristic("19B10014-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify); // acc.in z in g
BLEFloatCharacteristic gxCharacteristic("19B10015-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify); // rot.in x in deg/s
BLEFloatCharacteristic gyCharacteristic("19B10016-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify); // rot.in y in deg/s
BLEFloatCharacteristic gzCharacteristic("19B10017-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify); // rot.in z in deg/s
BLEIntCharacteristic shockCharacteristic("19B10018-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);  // true if shock sensed
BLEIntCharacteristic modeCharacteristic("19B10019-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);  // sensing mode
BLEIntCharacteristic srangeCharacteristic("19B10020-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);  // shock range
BLEIntCharacteristic arangeCharacteristic("19B10021-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);  // acc. range
BLEIntCharacteristic grangeCharacteristic("9B100122-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // gyro range
BLEIntCharacteristic calibCharacteristic("9B100123-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);  // 1 if calibration is on, otherwise 0



void setup()
{
   //Serial.begin(9600);
   //while (!Serial);    // wait for the serial port to open
  // Serial.println("Initializing IMU device...");
  CurieIMU.begin();
  // configure Bluetooth
  blePeripheral.setLocalName("MOOVBIT");
  blePeripheral.setAdvertisedServiceUuid(sensorService.uuid());
  blePeripheral.addAttribute(sensorService);
  blePeripheral.addAttribute(stepCharacteristic);
  blePeripheral.addAttribute(axCharacteristic);
  blePeripheral.addAttribute(ayCharacteristic);
  blePeripheral.addAttribute(azCharacteristic);
  blePeripheral.addAttribute(gxCharacteristic);
  blePeripheral.addAttribute(gyCharacteristic);
  blePeripheral.addAttribute(gzCharacteristic);
  // blePeripheral.addAttribute(shockCharacteristic);
  // blePeripheral.addAttribute(modeCharacteristic);
  blePeripheral.addAttribute(arangeCharacteristic);
  // blePeripheral.addAttribute(srangeCharacteristic);
  // blePeripheral.addAttribute(grangeCharacteristic);*/
  // blePeripheral.addAttribute(calibCharacteristic);
  // set initial characteristics values
  stepCharacteristic.setValue(0);
  shockCharacteristic.setValue(0);
  modeCharacteristic.setValue(0);
  arangeCharacteristic.setValue(4);
  srangeCharacteristic.setValue(4);
  grangeCharacteristic.setValue(250);
  calibCharacteristic.setValue(1);
  blePeripheral.begin();
  /*if (calibCharacteristic.value() == 1) {
    //Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);
    CurieIMU.autoCalibrateGyroOffset();
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  }
  else {
    //Serial.println("No calibration");
  }*/
  // Configure sensors 
  CurieIMU.setAccelerometerRange(arangeCharacteristic.value());
  CurieIMU.setGyroRange(grangeCharacteristic.value());
  CurieIMU.setStepDetectionMode(CURIE_IMU_STEP_MODE_SENSITIVE);
  CurieIMU.setStepCountEnabled(true);
  /*CurieIMU.setDetectionThreshold(CURIE_IMU_TAP, 750);       // (750mg)
  CurieIMU.setDetectionThreshold(CURIE_IMU_DOUBLE_TAP, 750);    // (750mg)
  CurieIMU.interrupts(CURIE_IMU_TAP);
  CurieIMU.interrupts(CURIE_IMU_DOUBLE_TAP);*/
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, srangeCharacteristic.value() * 500); // 1g = 1000 mg
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 50); // 50ms or 75ms
  CurieIMU.interrupts(CURIE_IMU_SHOCK);
  //CurieIMU.interrupts(CURIE_IMU_MOTION);
  CurieIMU.attachInterrupt(eventCallback);

}

void loop()
{
  if(!flag){
    CurieIMU.setAccelerometerRange(arangeCharacteristic.value());
    Serial.println(CurieIMU.getAccelerometerRange());
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
    flag = true;  
  }else{
    if(!axCharacteristic.setValue(convertRawAcceleration(aix))
     || !ayCharacteristic.setValue(convertRawAcceleration(aiy))
     || !azCharacteristic.setValue(convertRawAcceleration(aiz))
     || !gxCharacteristic.setValue(convertRawGyro(gix))
     || !gyCharacteristic.setValue(convertRawGyro(giy))
     || !gzCharacteristic.setValue(convertRawGyro(giz))){
      // do nothing
    }
    stepCharacteristic.setValue(CurieIMU.getStepCount());
    shockCharacteristic.setValue(0);
    flag = false;
  }
  delay(1000);
  /* NOT IMPLEMENTED WRITE CHARS SO COMMENTED THIS SECTION
  if (arangeCharacteristic.written()) CurieIMU.setAccelerometerRange(arangeCharacteristic.value());
  if (grangeCharacteristic.written()) CurieIMU.setGyroRange(grangeCharacteristic.value());
  if (srangeCharacteristic.written()) {
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, srangeCharacteristic.value() * 1000);
  }
  if (modeCharacteristic.written()) {
  // change sensing mode
  }*/

  /*if (motion) {
    if (!axCharacteristic.setValue(convertRawAcceleration(aix)) || !ayCharacteristic.setValue(convertRawAcceleration(aiy)) ||
      !azCharacteristic.setValue(convertRawAcceleration(aiz)) || !gxCharacteristic.setValue(convertRawGyro(gix)) ||
      !gyCharacteristic.setValue(convertRawGyro(giy)) || !gzCharacteristic.setValue(convertRawGyro(giz))) {
      //Serial.println("Erorr BLE");
    }
    stepCharacteristic.setValue(CurieIMU.getStepCount());
    motion = false;
  }
  
  shockCharacteristic.setValue(0);*/
}

void eventCallback() {
  if (CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK)) {
    shockCharacteristic.setValue(1);
  }

}

// converted values based on accelerometer range
float convertRawAcceleration(int aRaw) {
  float a = (aRaw * float(arangeCharacteristic.value()) / 32768.0); // output values from -16.0 to +16.0
  return a;
}

// converted values based on gyro range
float convertRawGyro(int gRaw) {
  float g = (gRaw * float(grangeCharacteristic.value()) / 32768.0); // output values from -2000.0 to +2000.0
  return g;
}
