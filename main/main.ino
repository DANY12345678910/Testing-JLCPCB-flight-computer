#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "BMI088.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <SoftwareSerial.h>

#define SEALEVELPRESSURE_HPA (1002.5)// http://www.monsieur-meteo.com/meteo/France/ile-de-france/yvelines/Versailles_132610.php

const int chipSelect = BUILTIN_SDCARD;
// Define the pin where the voltage divider is connected
const int voltagePin = 26;

// Define the reference voltage (usually 5V for Arduino Uno)
const float referenceVoltage = 5.0;
long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to Ublox module.


float all_data[15];


/* accel object */
Bmi088Accel accel(Wire,0x19);
/* gyro object */
Bmi088Gyro gyro(Wire,0x69);

Adafruit_BMP3XX bmp;

SFE_UBLOX_GPS myGPS;
void SD_setup(){
   Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  Serial.println("card initialized.");
}
void SD_Headers(){ ////à optimiser

  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print("Pressure");
    dataFile.print(",");
    dataFile.print("Temperature");
    dataFile.print(",");
    dataFile.print("Orientation X");
    dataFile.print(",");
    dataFile.print("Orientation Y");
    dataFile.print(",");
    dataFile.println("Orientation Z");
    dataFile.close();
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
   
}
void setup_IMU_BMI088(){
  int status;
  status = accel.begin();
  status=accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ); //Condition de Shannon, la fréquence d'échantillonage doit être 2x supérieure a la fréquence de lecture
  
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.begin();
  status = gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);//Condition de Shannon, la fréquence d'échantillonage doit être 2x supérieure a la fréquence de lecture
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }
} 
void setup_barometer_BMP388(){
   Serial.println("Adafruit BMP388");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
}
void setup_gps(){
  do {  //Assume that the U-Blox GPS is running at 9600 baud (the default) or at 38400 baud.
  //Loop until we're in sync and then ensure it's at 38400 baud.
    
    Serial8.begin(38400);
    if (myGPS.begin(Serial8) == true) break;

    delay(100);
   
    Serial8.begin(9600);
    if (myGPS.begin(Serial8) == true) {
         myGPS.setSerialRate(38400);
        delay(100);
    } else {
        //myGPS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);


  myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  SD_setup();
  SD_Headers();
  setup_IMU_BMI088();
  setup_barometer_BMP388();
  setup_gps();

}

void writeto_SD(float all_data[]){
  File dataFile = SD.open("datalog.csv", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    for(int i=0; i<15;i++){
      dataFile.print(all_data[i]);
      dataFile.print(',');

    }
    dataFile.println("");
    dataFile.close();
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.csv");
  }
}
void read_batt(){
  int rawValue = analogRead(voltagePin);

  // Convert the raw value to voltage using the reference voltage and ADC resolution
  float voltage = (float)rawValue * (referenceVoltage / 1023.0);

  // Print the voltage value to the serial monitor

  Serial.println(voltage*2.63157, 5); // Print with 2 decimal places

}
void read_IMU_BMI088(float all_data[]) 
{
  
  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  /* print the data */
  all_data[0]=accel.getAccelX_mss();
  all_data[1]=accel.getAccelY_mss();
  all_data[2]=accel.getAccelZ_mss();

  all_data[3]=gyro.getGyroX_rads();
  all_data[4]=gyro.getGyroY_rads();
  all_data[5]=gyro.getGyroZ_rads();

  all_data[6]=accel.getTemperature_C();
  
  delay(1);// delay de 1 milliseconde + 1 milliseconde le temps de la boucle= Fréquence de lecture a 500Hz
}
void read_barometer_BMP388(float all_data[]){
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
  }

  all_data[0]=bmp.temperature;
  all_data[1]=bmp.pressure / 100.0;
  all_data[2]=bmp.readAltitude(SEALEVELPRESSURE_HPA);
}
void read_gps(){
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 500)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGPS.getLatitude();//a voir si on perd la précision en castant le long en float
    Serial.print(F("1X2X3X"));
    Serial.print(latitude/10000000.0,10);

    long longitude = myGPS.getLongitude();
    Serial.print(F("X"));
    Serial.print(longitude/10000000.0,10);
    Serial.print(F("X"));

    long altitude = myGPS.getAltitude();
    
    Serial.print(altitude/1000.0,5);
    Serial.print(F("X"));

    byte SIV = myGPS.getSIV();
  
    Serial.print(SIV);

    Serial.println();
  }
}
void loop() {
  read_batt();
  writeto_SD(all_data);
  read_IMU_BMI088(all_data);
  read_barometer_BMP388(all_data);
  for(int i=0;i<7;i++){
    Serial.print(all_data[i]);
    Serial.print("\t");
  }
  
  Serial.print("\n");
  // put your main code here, to run repeatedly:

}
