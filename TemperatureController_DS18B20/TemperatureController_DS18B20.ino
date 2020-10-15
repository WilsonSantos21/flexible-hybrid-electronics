#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Adafruit_MAX31865.h>
#include <SD.h>

#include <Arduino.h>
#include <PID_v1.h>

//Fonte de tensão liga ao V+(vermelho) e GND(preto) do modulo MOS
//Peltier liga V+(vermelho) e V-(preto) do modulo MOS
/*
MOS module pinout:      PT100 - SPI pinout:       DS18B20 temperaure sensor:      SD Card - SPI pinout:
SIG - pin D9            VIN - 5V                  5V                              5V
GND                     GND                       GND                             GND
                        CLK - pin D5              DQ - pin D8                     CS - pin D10
                        SDO - pin D4                                              MOSI - pin D11
                        SDI - pin D3                                              MISO - pin D12
                        CS - pin D2                                               SCK - pin D13
*/

// All DS18B20 Sensors are connected to pin 8 on the Arduino
#define ONE_WIRE_BUS 8

// Creating a oneWire instance(object)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire object reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int numberOfDevices; //To store number of sensor connected
const int chipSelectSDCard = 10;

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 PT100 = Adafruit_MAX31865(2, 3, 4, 5); //PT100 definition

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

// IRF520 MOS Module
#define PWM_Mosfet 9  // pin that controls the MOSFET

// variable to store the value from the sensor reading
double temperatureMeasured; //PT100                  

// variables for output and setpoint
double valuePID;      //Peltier
double setTemperature; //desired temperature

//double Kp=5, Ki=3, Kd=3;
//double Kp=3, Ki=0.2, Kd=1;
double Kp=90, Ki=30, Kd=600;
int intervalTime = 20;
int intervalTime2 = 1000;
unsigned long previousTime;
unsigned long previousTime2;
float timeSeconds;

DeviceAddress tempDeviceAddress; // Variable to store a single sensor address

//PID instance
PID myPID(&temperatureMeasured, &valuePID, &setTemperature, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  while (!Serial);

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelectSDCard)) {
//    Serial.println("initialization failed. Things to check:");
//    Serial.println("1. is a card inserted?");
//    Serial.println("2. is your wiring correct?");
//    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
//    Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
    while (true);
  }

  Serial.println("initialization done.");

  sensors.begin();
  // Get the number of sensors connected to the the wire( digital pin 8)
  numberOfDevices = sensors.getDeviceCount();
  
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // Loop through each sensor and print out address
  for(int i=0; i<numberOfDevices; i++) {
    
    // Search the data wire for address and store the address in "tempDeviceAddress" variable
    if(sensors.getAddress(tempDeviceAddress, i)) {
      
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();     
    } else {
      
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
      
    }

      File dataFile = SD.open("datalog2.csv", FILE_WRITE);

      // if the file is available, write to it:
    if (dataFile) {
      dataFile.println("time(s),tempDS18B20(C),TemperatureDesired(C),PT100(C)");
      dataFile.close();
      // print to the serial port too:
      //Serial.println(tempC);
    }
      // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }    
  }
  
  PT100.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary

  // initial parameters to set temperature
  setTemperature = 30;

  myPID.SetMode(AUTOMATIC);

  pinMode(PWM_Mosfet, OUTPUT); // define control pin as output
}//End of void setup()


void loop() {

  unsigned long currentTime = millis();

  if(currentTime - previousTime2 >= intervalTime){

    myPID.Compute();
    //We define PWM range between 0 and 255
    if(valuePID < 0)
    {    valuePID = 0;    }
    if(valuePID > 255)  
    {    valuePID = 255;  }
    previousTime2 = currentTime;
    //First we read the real value of temperature
    temperatureMeasured = PT100.temperature(RNOMINAL, RREF);
    analogWrite(PWM_Mosfet, valuePID);
    
  }

  if(currentTime - previousTime >= intervalTime2){

    timeSeconds = currentTime/1000.0;
    
    sensors.requestTemperatures(); // Send the command to get temperatures from all sensors.

    sensors.setResolution(tempDeviceAddress, 12);
    // Loop through each device, print out temperature one by one
    for(int i=0; i<numberOfDevices; i++) {
      
      // Search the wire for address and store the address in tempDeviceAddress
      if(sensors.getAddress(tempDeviceAddress, i)){
      
//          Serial.print("Temperature from sensor number: ");
//          Serial.println(i,DEC);
  
        // Print the temperature
        float tempC = sensors.getTempC(tempDeviceAddress); //Temperature in degree celsius
        Serial.print("Time: ");
        Serial.print(timeSeconds);
        Serial.print(" s ");
        Serial.print("Temp C: ");
        Serial.print(tempC);
  
//  uint16_t rtd = PT100.readRTD();
//  float ratio = rtd;
//  ratio /= 32768;

//    Serial.print("Time: ");
//    Serial.print(timeSeconds);
//    Serial.print(" s ");
  Serial.print(" TemperatureDesired: ");
  Serial.print(setTemperature);
  Serial.print(" PT100: ");
  Serial.println(temperatureMeasured);
  
//    Serial.print(" valuePID: ");
//    Serial.println(valuePID);

    if(timeSeconds >= 120){
      setTemperature = 37;
    }

  // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog2.csv", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
  //    dataFile.print("Temp C: ");
      dataFile.print(timeSeconds);
      dataFile.print(",");
      dataFile.print(tempC);
      dataFile.print(",");
      dataFile.print(setTemperature);
      dataFile.print(",");
      dataFile.println(temperatureMeasured);
      dataFile.close();
      // print to the serial port too:
      //Serial.println(tempC);
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
      }   
    }
      previousTime = currentTime;
  }
}

/*----------------------------------------------------------------*/

// function to print a sensor address
void printAddress(DeviceAddress deviceAddress) {
  
  for (uint8_t i = 0; i < 8; i++) {
    
    if (deviceAddress[i] < 16) 
      Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
      
  }//End of for loop
  
}

/*----------------------------------------------------------------*/
