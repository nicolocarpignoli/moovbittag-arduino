/*

Sketch for automated unit test in Genuino101
@author Nicolo Carpignoli

*/

#include <ArduinoUnit.h>
#include "CurieIMU.h"
#include "CurieBLE.h"


bool motion = false;
int aix, aiy, aiz;		// raw accelerometer values
int gix, giy, giz;		// raw gyro values

BLEPeripheral blePeripheral;
BLEService sensorService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLEUnsignedLongCharacteristic stepCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead);	// stepcount
BLEFloatCharacteristic axCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);	// acc.in x in g
BLEFloatCharacteristic ayCharacteristic("19B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);	// acc.in y in g
BLEFloatCharacteristic azCharacteristic("19B10014-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);	// acc.in z in g
BLEFloatCharacteristic gxCharacteristic("19B10015-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);	// rot.in x in deg/s
BLEFloatCharacteristic gyCharacteristic("19B10016-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);	// rot.in y in deg/s
BLEFloatCharacteristic gzCharacteristic("19B10017-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);	// rot.in z in deg/s
BLEIntCharacteristic shockCharacteristic("19B10018-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);	// true if shock sensed
BLEIntCharacteristic modeCharacteristic("19B10019-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);	// sensing mode
BLEIntCharacteristic srangeCharacteristic("19B10020-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);	// shock range
BLEIntCharacteristic arangeCharacteristic("19B10021-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);	// acc. range
BLEIntCharacteristic grangeCharacteristic("19B100122-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);	// gyro range
BLEIntCharacteristic calibCharacteristic("19B100123-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);	// 1 if calibration is on, otherwise 0



// unit tests

test(setup)
{
	real_setup();
	assertEqual(stepCharacteristic.value(), 0);
	assertEqual(shockCharacteristic.value(), 0);
	assertEqual(modeCharacteristic.value(), 0);
	assertEqual(arangeCharacteristic.value(), 4);
	assertEqual(srangeCharacteristic.value(), 8);
	assertEqual(grangeCharacteristic.value(), 250);
	assertEqual(calibCharacteristic.value(), 1);
	assertEqual(CurieIMU.getAccelerometerRange(), arangeCharacteristic.value());
	assertEqual(CurieIMU.getGyroRange(), grangeCharacteristic.value());
	assertEqual(CurieIMU.getStepCountEnabled(), true);
}

test(converter_acc)
{
	arangeCharacteristic.setValue(4);
	float a = (8192 * float(arangeCharacteristic.value()) / 32768.0);
	assertEqual(a, float(1));
	assertNotEqual(a, 2);
}

test(converter_gyro) 
{
	grangeCharacteristic.setValue(8);
	float a = (4096 * float(grangeCharacteristic.value()) / 32768.0);
	assertEqual(a, float(1))
	assertNotEqual(a, 2);
}


// functions to test
// copied here and renamed for avoid problems with arduino sketches' structure

void real_setup() {
	Serial.begin(9600);
	while (!Serial);    // wait for the serial port to open
	CurieIMU.begin();
	// configure Bluetooth
	blePeripheral.setLocalName("CurieSensor");
	blePeripheral.setAdvertisedServiceUuid(sensorService.uuid());
	blePeripheral.addAttribute(sensorService);
	blePeripheral.addAttribute(stepCharacteristic);
	blePeripheral.addAttribute(axCharacteristic);
	blePeripheral.addAttribute(ayCharacteristic);
	blePeripheral.addAttribute(azCharacteristic);
	blePeripheral.addAttribute(gxCharacteristic);
	blePeripheral.addAttribute(gyCharacteristic);
	blePeripheral.addAttribute(gzCharacteristic);
	blePeripheral.addAttribute(shockCharacteristic);
	blePeripheral.addAttribute(modeCharacteristic);
	blePeripheral.addAttribute(arangeCharacteristic);
	blePeripheral.addAttribute(srangeCharacteristic);
	blePeripheral.addAttribute(grangeCharacteristic);
	blePeripheral.addAttribute(calibCharacteristic);
	// set initial characteristics values
	stepCharacteristic.setValue(0);
	shockCharacteristic.setValue(0);
	modeCharacteristic.setValue(0);
	arangeCharacteristic.setValue(4);
	srangeCharacteristic.setValue(8);
	grangeCharacteristic.setValue(250);
	calibCharacteristic.setValue(1);
	//real_initPeriph();
	if (calibCharacteristic.value() == 1) {
		CurieIMU.autoCalibrateGyroOffset();
		CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
		CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
		CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
	}
	// Configure sensors 
	CurieIMU.setAccelerometerRange(arangeCharacteristic.value());
	CurieIMU.setGyroRange(grangeCharacteristic.value());
	CurieIMU.setStepDetectionMode(CURIE_IMU_STEP_MODE_NORMAL);
	CurieIMU.setStepCountEnabled(true);
	CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, srangeCharacteristic.value() * 1000);	// 1g = 1000 mg
	CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 50);	// 50ms or 75ms
	CurieIMU.interrupts(CURIE_IMU_SHOCK);
	CurieIMU.interrupts(CURIE_IMU_MOTION);
	//CurieIMU.attachInterrupt(real_eventCallback);
}


// converted values based on accelerometer range
float real_convertRawAcceleration(int aRaw) {
	float a = (aRaw * float(arangeCharacteristic.value()) / 32768.0);	// output values from -16.0 to +16.0
	return a;
}

// converted values based on gyro range
float real_convertRawGyro(int gRaw) {
	float g = (gRaw * float(grangeCharacteristic.value()) / 32768.0);	// output values from -2000.0 to +2000.0
	return g;
}




void setup()
{
	Serial.begin(9600);
	while (!Serial); // for the Arduino Leonardo/Micro only
}

void loop()
{
	Test::run();
}