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
float tempC;
//int msb;
//int lsb;

// All DS18B20 Sensors are connected to pin 8 on the Arduino
#define ONE_WIRE_BUS 8

// Creating a oneWire instance(object)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire object reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int numberOfDevices; //To store number of sensor connected

DeviceAddress tempDeviceAddress; // Variable to store a single sensor address

/*----------------------------------------------------------------*/

void setup(void) {
 
  Serial.begin(115200);
  sensors.begin();
  
  // Get the number of sensors connected to the the wire( digital pin 8)
  numberOfDevices = sensors.getDeviceCount();
  
//  Serial.print(numberOfDevices, DEC);
//  Serial.println(" devices.");

  // Loop through each sensor and print out address
  for(int i=0; i<numberOfDevices; i++) {
    
    // Search the data wire for address and store the address in "tempDeviceAddress" variable
    if(sensors.getAddress(tempDeviceAddress, i)) {
      
//      Serial.print("Found device ");
//      Serial.print(i, DEC);
//      Serial.print(" with address: ");
//      printAddress(tempDeviceAddress);
//      Serial.println();
      
    } else {
      
//      Serial.print("Found ghost device at ");
//      Serial.print(i, DEC);
//      Serial.print(" but could not detect address. Check power and cabling");
      
    }
    
  }//Enf of for loop

  if(!ntag.begin()){
//        Serial.println("Can't find ntag");
    }
    ntag.detectI2cDevices();
    //testWriteAdapter();
    getSerialNumber();
    
}//End of void setup()

/*----------------------------------------------------------------*/

void loop(void) { 
  //sensors.setResolution(tempDeviceAddress, 12);
  sensors.requestTemperatures(); // Send the command to get temperatures from all sensors.
  
  // Loop through each device, print out temperature one by one
  for(int i=0; i<numberOfDevices; i++) {
    
    // Search the wire for address and store the address in tempDeviceAddress
    if(sensors.getAddress(tempDeviceAddress, i)){
    
//      Serial.print("Temperature from sensor number: ");
//      Serial.println(i,DEC);

      
      // Print the temperature
      tempC = sensors.getTempC(tempDeviceAddress); //Temperature in degree celsius
      Serial.println(tempC);
      //delay(1000);
      testSram();
    }   
    
  }// End of for loop

  //delay(1000);
  
  
  
  //delay(1000);
}

/*----------------------------------------------------------------*/
void testSram(){
    
    byte data[16];
    int msb = sensors.getMSBTemp();
    int lsb = sensors.getLSBTemp();
    Serial.print(msb);
    Serial.println(lsb);
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
    data[0] = msb;
    data[1] = lsb;
    for(byte i=2;i<16;i++){
        data[i] = lsb;
    }

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

// function to print a sensor address
//void printAddress(DeviceAddress deviceAddress) {
//  
//  for (uint8_t i = 0; i < 8; i++) {
//    
//    if (deviceAddress[i] < 16) 
////      Serial.print("0");
////      Serial.print(deviceAddress[i], HEX);
//      
//  }//End of for loop
//  
//}

void getSerialNumber(){
    byte* sn=(byte*)malloc(ntag.getUidLength());
//    Serial.println();
    if(ntag.getUid(sn,7))
    {
//        Serial.print("Serial number of the tag is: ");
        for(byte i=0;i<ntag.getUidLength();i++)
        {
//            Serial.print(sn[i], HEX);
//            Serial.print(" ");
        }
    }
//    Serial.println();
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
