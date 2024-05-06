#include <Arduino.h>

#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "BMI088.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Wire.h> //Needed for I2C to GNSS
#include <SoftwareSerial.h>
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include <CayenneLPP.h>
#include <RadioLib.h>


#define SEALEVELPRESSURE_HPA (1002.5)// http://www.monsieur-meteo.com/meteo/France/ile-de-france/yvelines/Versailles_132610.php
// Définir les broches utilisées pour la communication LoRa
#define LORA_MOSI 11
#define LORA_MISO 12
#define LORA_SCK 13
#define LORA_NSS 10
#define LORA_DIO0 9
#define LORA_RST 8

#define DIO1 2
#define NRST 3
#define BUSY 9
#define rxe 4
#define txe 5
// carrier frequency:           915.0 MHz
  // bandwidth:                   500.0 kHz
  // spreading factor:            6
  // coding rate:                 5
  // sync word:                   0x34 (public network/LoRaWAN)
  // output power:                20 dBm
  // preamble length:             20 symbols
#define FREQUENCY 868
#define BANDWIDTH 125
#define SPREADINGFACTOR 9
#define CODINGRATE 5
#define SYNCWORD 0x34
#define POWER 20
#define PLENGTH 20

// Définir les constantes utilisées pour le stockage des données
#define NB_DATA 15
#define DEBUGGING 1 //set to 0 if you don't want any Serial.Prints
#define GS_PRINT 1 //set to 1 if you don't want data values 1X2X3.... Printed (FLIGHT COMPUTER WIRED TO GS YES?=1)

const int chipSelect = BUILTIN_SDCARD;
// Define the pin where the voltage divider is connected
const int voltagePin = 26;

// Define the reference voltage (usually 5V for Arduino Uno)
const float referenceVoltage = 5.0;
long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to Ublox module.


float all_data[NB_DATA];


/* accel object */
Bmi088Accel accel(Wire,0x19);
/* gyro object */
Bmi088Gyro gyro(Wire,0x69);

Adafruit_BMP3XX bmp;

SFE_UBLOX_GNSS myGPS;

SX1262 radio = new Module(LORA_NSS, DIO1, NRST, BUSY);

void SD_setup(){
  if(DEBUGGING==1){
  Serial.print("Initializing SD card...");
  }
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    if(DEBUGGING==1){
      Serial.println("Card failed, or not present");
    }
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  if(DEBUGGING==1){
    Serial.println("card initialized.");
  }
}
void SD_Headers(){// Ouverture du fichier "Headers.txt" en mode lecture
  File headersFile = SD.open("Headers.txt", FILE_READ);
  
  if (headersFile) {
    // Lecture de la première ligne du fichier "Headers.txt"
    String headers = headersFile.readStringUntil('\n');
    headersFile.close(); // Fermeture du fichier "Headers.txt"

    // Ouverture du fichier "datalog.csv" en mode écriture (écrasement du contenu existant)
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    
    if (dataFile) {
      // Écriture de la première ligne (headers) dans le fichier "datalog.csv"
      dataFile.println(headers);
      
      // Fermeture du fichier "datalog.csv"
      dataFile.close();
    } else {
      // Erreur lors de l'ouverture du fichier "datalog.csv"
      Serial.println("Error opening datalog.csv");
    }
  } else {
    // Erreur lors de l'ouverture du fichier "Headers.txt"
    Serial.println("Error opening Headers.txt");
  }
}
void setup_IMU_BMI088(){
  int status;
  status = accel.begin();
  status=accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ); //Condition de Shannon, la fréquence d'échantillonage doit être 2x supérieure a la fréquence de lecture
  if(DEBUGGING==1){
    Serial.println("Setting up BMI088");
  }
  if (status < 0) {
    if(DEBUGGING==1){
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    }
    while (1) {}
  }
  status = gyro.begin();
  status = gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);//Condition de Shannon, la fréquence d'échantillonage doit être 2x supérieure a la fréquence de lecture
  if (status < 0) {
    if(DEBUGGING==1){
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    }
    while (1) {}
  }
} 
void setup_barometer_BMP388(){
  if(DEBUGGING==1){
  Serial.println("Adafruit BMP388 setup");
  }
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  if(DEBUGGING==1){
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
}
void setup_gps(){
  if(DEBUGGING==1){
    Serial.println("Setting up GPS");
  }
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
void setupLoRa(){
  if(DEBUGGING==1){
  Serial.print(F("[SX1262] Initializing ... "));
  }
  int state = radio.begin(FREQUENCY,BANDWIDTH,SPREADINGFACTOR,CODINGRATE,SYNCWORD,POWER,PLENGTH);
  if (state == RADIOLIB_ERR_NONE) {
    if(DEBUGGING==1){
    Serial.println(F("success!"));
    }
  } else {
    if(DEBUGGING==1){
    Serial.print(F("failed, code "));
    Serial.println(state);
    }
    while (true);
  }
  // to enable automatic control of the switch,
  // call the following method
  // RX enable:   4
  // TX enable:   5
  
  radio.setRfSwitchPins(rxe,txe);
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
  //setupLoRa();

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
    if(DEBUGGING==1){
    Serial.println("error opening datalog.csv");
    }
  }
}
void read_batt(){
  int rawValue = analogRead(voltagePin);

  // Convert the raw value to voltage using the reference voltage and ADC resolution
  float voltage = (float)rawValue * (referenceVoltage / 1023.0);
  all_data[13]=voltage*2.63157;
}
void read_IMU_BMI088(float all_data[]) 
{
  
  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  /* print the data */
  all_data[6]=accel.getAccelX_mss();
  all_data[7]=accel.getAccelY_mss();
  all_data[8]=accel.getAccelZ_mss();

  all_data[9]=gyro.getGyroX_rads();
  all_data[10]=gyro.getGyroY_rads();
  all_data[11]=gyro.getGyroZ_rads();

  all_data[12]=accel.getTemperature_C();
  
  //delay(1);// delay de 1 milliseconde + 1 milliseconde le temps de la boucle= Fréquence de lecture a 500Hz
}
void read_barometer_BMP388(float all_data[]){
  if (! bmp.performReading()) {
    if(DEBUGGING==1){
    Serial.println("Failed to perform reading :(");
    }
  }

  all_data[3]=bmp.temperature;
  all_data[4]=bmp.pressure / 100.0;
  all_data[5]=bmp.readAltitude(SEALEVELPRESSURE_HPA);
}
void read_gps(float all_data[]){
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available

    
    long latitude = myGPS.getLatitude();//a voir si on perd la précision en castant le long en float
    all_data[0]=float(latitude/10000000.0);
    //Serial.println(latitude,5,10);

    long longitude = myGPS.getLongitude();
    all_data[1]=float(longitude/10000000.0);
    //Serial.println(longitude/10000000.0,10);
 

    long altitude = myGPS.getAltitude();
    all_data[2]=float(altitude/1000.0);
    //Serial.println(altitude/1000.0,5);


    //byte SIV = myGPS.getSIV();
    //Serial.print("SIV: ");
    //Serial.println(SIV);

  }

void send_packet(float all_data[]){
  if(DEBUGGING==1){
  Serial.print(F("[SX1262] Transmitting packet ... "));
  }
  // Créer une instance de CayenneLPP
  CayenneLPP lpp(51);

  // Ajouter les données GPS
  lpp.addGPS(1,all_data[0], all_data[1], all_data[2]);

  // Ajouter les données de l'accéléromètre
  lpp.addAccelerometer(2, all_data[6], all_data[7], all_data[8]);

  // Ajouter les données du gyroscope
  lpp.addGyrometer(3, all_data[9], all_data[10], all_data[11]);

  // Ajouter les données du baromètre
  lpp.addBarometricPressure(4, all_data[4]);

  // Ajouter les données de la température
  lpp.addTemperature(5, all_data[3]);


  // Ajouter les données de l'altitude
  lpp.addAltitude(6, all_data[5]);

  // Ajouter les données de la tension de la batterie
  lpp.addAnalogInput(7, all_data[13]);

  // Ajouter les données de la température BMI088
  lpp.addTemperature(8, all_data[12]);
  // Obtenir la taille du payload CayenneLPP
  int lppSize = lpp.getSize();

  // Créer un tableau de bytes pour stocker le payload CayenneLPP
  byte lppBuffer[lppSize];

  // Copier le payload CayenneLPP dans le tableau de bytes
  memcpy(lppBuffer, lpp.getBuffer(), lppSize);

  // Envoyer le payload CayenneLPP via LoRa
  int state = radio.transmit(lppBuffer, lppSize);

  // Vérifier l'état de l'envoi
  if (state == RADIOLIB_ERR_NONE) {
    // l'envoi a réussi
    if(DEBUGGING==1){
    Serial.println(F("success!"));

    // Afficher le débit de données
    Serial.print(F("[SX1262] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));
    }
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // le paquet est trop long
    if(DEBUGGING==1){
    Serial.println(F("too long!"));
    }

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout d'envoi
    if(DEBUGGING==1){
    Serial.println(F("timeout!"));
    }

  } else {
    // autre erreur
    if(DEBUGGING==1){
    Serial.print(F("failed, code "));
    Serial.println(state);
    }

  }
}


void loop() {
  read_batt();
  writeto_SD(all_data);
  read_IMU_BMI088(all_data);
  read_barometer_BMP388(all_data);

  static uint32_t last_transmit = 0;
  //if (millis() - last_transmit >= 1000) {//TROP LENT
    //last_transmit = millis();
    //read_gps(all_data);
    //send_packet(all_data);
  //}
  if(GS_PRINT==1){
  Serial.print("1X2X3X");
  for(int i=0;i<NB_DATA;i++){
    Serial.print(all_data[i],10);
    Serial.print("X");
  }
  Serial.print("1");
  Serial.println();
  }
}
