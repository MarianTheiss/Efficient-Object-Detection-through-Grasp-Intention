#include <RFduinoBLE.h>
#include <Wire.h>
#include "skyetek_m1.h"

M1 nfc(5,6);

void setup()
{
  //uint32_t versiondata = nfc.getFirmwareVersion();
  //if (!versiondata)
  //  while(1);

  RFduinoBLE.deviceName = "RFIDReader";
  RFduinoBLE.begin();
}

void check_for_tag(char *data)
{
   uint8_t uid[80], len=0;
  if (len=nfc.selectTag(uid+2,sizeof(uid)-2))// -first byte is tag type +2 byte ID
  {
    uid[0]=data[0];
    uid[1]=data[1];
    RFduinoBLE.send((char*) &uid, len+2); // -first byte is tag type +2 byte ID
  }
  else
  {
    uid[0]=data[0];
    uid[1]=data[1];
    RFduinoBLE.send((char*) &uid, 2); // -first byte is tag type +2 byte ID
  }
}

void loop()
{
  //RFduinoBLE.send((char*) 11, 11);
  RFduino_ULPDelay(INFINITE);
}

void RFduinoBLE_onConnect()
{
}

void RFduinoBLE_onDisconnect()
{
}

void RFduinoBLE_onReceive(char *data, int len)
{
  //digitalWrite(led, HIGH);
  // if the first byte is 0x01 / on / true
  if (sizeof(data)>1)
  {
    //digitalWrite(led, HIGH);
    check_for_tag(data); 
  }
  else
  {
    //digitalWrite(led, LOW); 
  }   
}
