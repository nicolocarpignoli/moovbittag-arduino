#include "CurieIMU.h"
#include "CurieBLE.h"
#include "HexConversionUtils.h"
#include "SipHash_2_4.h"
#include <stdio.h>      /* puts, printf */

int aix, aiy, aiz;    // raw accelerometer values
int gix, giy, giz;    // raw gyro values
bool flag = false;
bool isAllowedFlag = false;
long localDigest = -1;
char digestHex[17];
long firstHalf;
long secondHalf; 
boolean readyForTheSecond = false;
String firstHalfString;
String secondHalfString;
String trustedAddress;
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
BLELongCharacteristic authCharacteristic("19B100124-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);  // auth_characteristic

 

void setup()
{
   Serial.begin(9600);
  // while (!Serial);    // wait for the serial port to open
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
  blePeripheral.addAttribute(shockCharacteristic);
  blePeripheral.addAttribute(modeCharacteristic);
  blePeripheral.addAttribute(arangeCharacteristic);
  blePeripheral.addAttribute(srangeCharacteristic);
  blePeripheral.addAttribute(grangeCharacteristic);
  blePeripheral.addAttribute(authCharacteristic);
  // set initial characteristics values
  stepCharacteristic.setValue(0);
  shockCharacteristic.setValue(0);
  modeCharacteristic.setValue(0);
  arangeCharacteristic.setValue(4);
  srangeCharacteristic.setValue(4);
  grangeCharacteristic.setValue(250);
  authCharacteristic.setValue(-1);
  blePeripheral.begin();
  CurieIMU.setAccelerometerRange(arangeCharacteristic.value());
  CurieIMU.setStepDetectionMode(CURIE_IMU_STEP_MODE_SENSITIVE);
  CurieIMU.setStepCountEnabled(true);
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, srangeCharacteristic.value() * 500); // 1g = 1000 mg
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 50); // 50ms or 75ms
  CurieIMU.interrupts(CURIE_IMU_SHOCK);
  CurieIMU.attachInterrupt(eventCallback);
  uint8_t key[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                        0x18, 0x19, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
  uint8_t message[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                        0x18, 0x19, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e};
  int msgLen = 15;
  sipHash.initFromPROGMEM(key);
  for (int j=0; j<msgLen;j++) {
      sipHash.updateHash((byte)message[j]);
    }
  sipHash.finish(); // result in BigEndian format
  hexToAscii(sipHash.result,8,digestHex,17);
 }

void loop()
{
//ARDUINO INIT:
  BLECentral bleCentral = blePeripheral.central();
  if (bleCentral) {
     while(bleCentral.connected()){
           String address = bleCentral.address();
           boolean macClientIsAllowed = checkMac(address, trustedAddress);
           if(isAllowedFlag || macClientIsAllowed){
                  if(!flag){
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
            }
           if(authCharacteristic.written()){ //CHECK IF MAC IS ALLOWED
                       //ARDUINO WAIT FOR THE SECOND
                       Serial.println("Auth is modified by: ");
                       Serial.println(address);
                       if(!readyForTheSecond){ 
                              firstHalf = authCharacteristic.value();
                              firstHalfString = String(firstHalf, 16);
                              authCharacteristic.setValue(0);
                              readyForTheSecond = true;
                       }else { 
                              readyForTheSecond = false;
                              secondHalf = authCharacteristic.value();
                              secondHalfString = String(secondHalf, 16);
                            if (checkForAllowing(firstHalfString,secondHalfString)){
                                      isAllowedFlag = true;
                                      Serial.println("allowed: ");
                                      trustedAddress= address;
                                      Serial.println(address);
                                      Serial.println(axCharacteristic.value());
                              } else {
                                isAllowedFlag = false;
                                Serial.println("not allowed:" );
                                Serial.println(address);
                              }
                        }
                }
            if(arangeCharacteristic.written()){
                  Serial.println("I write the arangeChar");
                  if ((isAllowedFlag == false) && (macClientIsAllowed == false)){
                       Serial.println("i'm not allowed to write");
                       arangeCharacteristic.setValue(0);
                       Serial.println(arangeCharacteristic.value());   
                 } else {
                    Serial.println("i'm allowed to write");
                    Serial.println(arangeCharacteristic.value());
                 }
           } 
           
          }
 }
 isAllowedFlag = false;
}
 
 

boolean checkForAllowing(String firstHalf,String secondHalf){
  String digest = secondHalf + firstHalf;
  Serial.println("External digest is: ");
  Serial.println(digest);
  String localDigest = String(digestHex);
  Serial.println("local digest is: ");
  Serial.println(localDigest);
  return localDigest.equalsIgnoreCase(digest);
}


boolean checkMac(String newMac, String trustedMac){
  return newMac.equalsIgnoreCase(trustedMac);
}



void eventCallback() {
  if (CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK)) {
    shockCharacteristic.setValue(1);
  }
}




void printKey(unsigned char PROGMEM *key) {
  Serial.print(F(" Key:"));
  unsigned char tmpKey[16];
  memcpy_P(tmpKey,key,16);
  char tmp[33];
  hexToAscii((const unsigned char*)tmpKey,16,tmp,33);
  Serial.println(tmp);
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
