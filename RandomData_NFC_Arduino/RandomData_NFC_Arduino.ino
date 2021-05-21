#include <OneWire.h>
#include "DallasTemperature.h"
#include "ntagsramadapter.h"
#include "Arduino.h"
#define HARDI2C
#include <Wire.h>

Ntag ntag(Ntag::NTAG_I2C_2K,7,9);
//NtagSramAdapter ntagAdapter(&ntag);
int current_time;
long randNumber;
byte data;
long previousMillis = 0;        // Variável de controle do tempo
long redLedInterval = 0;     // Tempo em ms do intervalo a ser executado

/*----------------------------------------------------------------*/

void setup(void) {
Serial.begin(115200);
    randomSeed(analogRead(0));
    Serial.println("---");
    Serial.println("start");
    if(!ntag.begin()){
        Serial.println("Can't find ntag");
    }
    ntag.detectI2cDevices();
    //testWriteAdapter();
    //getSerialNumber();
    //testUserMem();
    //testRegisterAccess();
    //testSramMirror();
    //testSram();
    Serial.println("---");
    pinMode(LED_BUILTIN, OUTPUT);
}//End of void setup()

/*----------------------------------------------------------------*/

void loop(void) {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    
//      Serial.print("Temperature from sensor number: ");
//      Serial.println(i,DEC);

      //delay(1000);
      testSram();   
    
  }// End of for loop

  //delay(1000);
  
  
  
  //delay(1000);

/*----------------------------------------------------------------*/
void testSram(){
    
    byte data[16];
//    ntag.writeRegister(Ntag::NC_REG, 0x7C, 0x7C);
    
    //Serial.print("NC_REG_NFCtoI2C: ");
//    ntag.readRegister(Ntag::NC_REG,data[0]);
    //Serial.println(data[0],HEX);

    //Serial.print("NS_REG_NFCtoI2C: ");
//    ntag.readRegister(Ntag::NS_REG,data[0]);
    //Serial.println(data[0],HEX);

//    delay(500);
    //Serial.println("Reading SRAM block 0xF8");
//    if(ntag.readSram(0,data,16)){
//        showBlockInHex(data,16);
//   }

    for(byte i=0;i<16;i++){
        //data[i] = 0xF0 | i;
        randNumber = random(100);
        data[i] = randNumber;
    }
    Serial.println(data[0]);

    ntag.writeRegister(Ntag::NC_REG, 0x7D, 0x7C);
    
    //Serial.println("Writing dummy data to SRAM block 0xF8");
    if(!ntag.writeSram(0,data,16)){
        return;
    }
    ntag.releaseI2c();

    //Serial.print("NC_REG_I2CtoNFC: ");
    ntag.readRegister(Ntag::NC_REG,data[0]);
    //Serial.println(data[0],HEX);

    //Serial.print("NS_REG_I2CtoNFC: ");
    ntag.readRegister(Ntag::NS_REG,data[0]);
    //Serial.println(data[0],HEX);

    delay(500);
    
//    ntag.writeRegister(Ntag::NC_REG, 0x01, 0x01);
}

void getSerialNumber(){
    byte* sn=(byte*)malloc(ntag.getUidLength());
    Serial.println();
    if(ntag.getUid(sn,7))
    {
        Serial.print("Serial number of the tag is: ");
        for(byte i=0;i<ntag.getUidLength();i++)
        {
            Serial.print(sn[i], HEX);
            Serial.print(" ");
        }
    }
    Serial.println();
    free(sn);
}

void showBlockInHex(byte* data, byte size){
    for(int i=0;i<size;i++){
        Serial.print(data[i]/*,HEX*/);
        Serial.print(" ");
    }
    Serial.println();
}
/*----------------------------------------------------------------*/
