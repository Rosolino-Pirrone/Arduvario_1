#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEGatt.h"

#include "BluefruitConfig.h"
#include "IEEE11073float.h"
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEGatt gatt(ble);
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */
int32_t htsServiceId;
int32_t htsMeasureCharId;
int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmMeasureCharId2;
int32_t hrmLocationCharId;




#define GPSSerial Serial1
     
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <STRING.h>
int request;
char dati8[24];
String Vario_al_secondo;
float Vario;
float Tono_beep = 0;
float Durata_beep;
float Tono_beep_disc;

volatile uint32_t realTcount = 0x0 ;   // Counter for superticks (overflow interrupts)
//SoftwareSerial GPSSerial(2, 3);

//#define GPSSerial Serial
/*   -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

Adafruit_GPS GPS(&GPSSerial);
     

#define GPSECHO true

uint32_t timer = millis();


void setup()
{
  delay(500);

 boolean success;

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Health Thermometer Example"));
  Serial.println(F("--------------------------------------------"));

  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Perform a factory reset to make sure everything is in a known state */
  /*
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }
*/
  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  /*while (! ble.isConnected()) {
      delay(500);
  }*/

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }



  

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit HRM': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Arduvario")) ) {
    error(F("Could not set device name?"));
  }


/*
Serial.println(F("Adding the Health Thermometer Service definition (UUID = 0x1809): "));
  htsServiceId = gatt.addService(0x180D);
  if (htsServiceId == 0) {
    error(F("Could not add Thermometer service"));
  }
  */
  /* Add the Temperature Measurement characteristic which is composed of
   * 1 byte flags + 4 float */
  /* Chars ID for Measurement should be 1 */
  /*
  Serial.println(F("Adding the Temperature Measurement characteristic (UUID = 0x2A1C): "));
  //htsMeasureCharId = gatt.addCharacteristic(0x2A6D, GATT_CHARS_PROPERTIES_NOTIFY, 5, 5, BLE_DATATYPE_BYTEARRAY);
  htsMeasureCharId = gatt.addCharacteristic(0x2A6D, GATT_CHARS_PROPERTIES_NOTIFY, 5, 20, BLE_DATATYPE_STRING);
  if (htsMeasureCharId == 0) {
    error(F("Could not add Temperature characteristic"));
  }
*/
  /* Add the Health Thermometer Service to the advertising data (needed for Nordic apps to detect the service) */
 /* 
  Serial.print(F("Adding Health Thermometer Service UUID to the advertising payload: "));
  uint8_t advdata[] { 0x02, 0x01, 0x06, 0x05, 0x02, 0x0d, 0x18, 0x0a, 0x18 };
  ble.setAdvData( advdata, sizeof(advdata) );
*/
  /* Reset the device for the new service setting changes to take effect */
 /* 
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();
*/
  Serial.println();


  

  


  
  
  //NMEA_vario.reserve(24);
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  setTC3clock();
  //Serial.begin(115200);
  //Serial.println("Adafruit GPS library basic test!");
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent);   
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}


void loop() // run over and over again
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  //if (GPSECHO)
    //if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
   // Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 1000) {//Serial.println(millis() - timer > 1000);
    timer = millis(); // reset the timer
    String fix;
   /* if ((int)GPS.fix == 1)
    {
    fix = "A";
    }
    fix = "V";
    String Time = String(GPS.time, DEC);
   // String Latitude = (String(GPS.latitude, 4) + "," + String(GPS.lat));
   // String Longitude = (String(GPS.longitude, 4) + "," + String(GPS.lon));
    String Speed = String(GPS.speed);
    String Angle = String(GPS.angle);
    String Date = String(GPS.fulldate, DEC);
    String stringa_nmea_GGA = GPS.stringa_GGA; 
    String stringa_nmea_RMC = GPS.stringa_RMC;
   // String GPS_NMEA = "GPRMC," + Time + "," + fix + "," + Latitude + "," + Longitude + "," + Speed + "," + Angle + "," + Date + "," + "," + ",";
    char check = 0;
    for (int c = 0; c < GPS_NMEA.length(); c++) {
    check = int(check ^ GPS_NMEA.charAt(c));
    String checkSum0 = String(check, HEX);
    }
   // Serial.println("boo$" + GPS_NMEA + "*" + checkSum0);              //Stringa alla seriale  
  */
  /*
 String stringa_nmea_GGA = GPS.stringa_GGA; 
    String stringa_nmea_RMC = GPS.stringa_RMC;
   
String NMEA_POV_1 = dati8;
NMEA_POV_1.trim();
//Serial.println(NMEA_POV_1);
int x = NMEA_POV_1.indexOf(",");
NMEA_POV_1.remove(x);
Vario_al_secondo = NMEA_POV_1;


String NMEA_POV_2 = dati8;
NMEA_POV_2.trim();
x = NMEA_POV_2.indexOf(",");
NMEA_POV_2.remove(0, (x+1)) ;
x = NMEA_POV_2.indexOf(",");
NMEA_POV_2.remove(x);
String Media_P = NMEA_POV_2;


String NMEA_POV_3 = dati8;
NMEA_POV_3.trim();
x = NMEA_POV_3.indexOf(",");
NMEA_POV_3.remove(0, (x+1)) ;
x = NMEA_POV_3.indexOf(",");
NMEA_POV_3.remove(0, (x+1)) ;
String Temp = NMEA_POV_3; 
Vario = Vario_al_secondo.toInt();
Vario = Vario/10;
 //Serial.println(NMEA_RMC);
   String cmd = "POV,E," + String(Vario) + ",P," + Media_P + ",T," + Temp;   // calcolo stringa NMEA OpenVario
  String checkSum1 = String(checkSum(cmd), HEX);
  
 Serial.print("$" + cmd + "*" + checkSum1);              //Stringa alla seriale
 Serial.print(stringa_nmea_GGA);
 Serial.println(stringa_nmea_RMC);
 //String blue_Stringa = ("$" + cmd + "*" + checkSum1);
 String blue_Stringa = ("$" + cmd + "*" + checkSum1);           //("$POV,E,-0.3,P,934.98,T,28.82*37");  // ("-0.3,934.98,28.82");
 //Serial.print(String(GPS.fix) + "\n");
  float temp_1 = Media_P.toFloat()*100;
  uint32_t temp = uint32_t(temp_1);
  //if (temp<0) {temp =0;}
Serial.println(temp);
 uint8_t temp_measurement [5] = { 0 };
 temp_measurement [4] = { temp>>24 };
 temp_measurement [3] = { temp>>16 & 0xFF };
 temp_measurement [2] = { temp>>8 & 0xFF };
 temp_measurement [1] = { temp & 0xFF };
  //float2IEEE11073(temp, temp_measurement+1);
  int n_nmea_blue = blue_Stringa.length();
  char dati_blue[n_nmea_blue+1];*/
//blue_Stringa.toCharArray(dati_blue, n_nmea_blue+1);
//ble.print("AT+BLEUARTTX=");
//ble.println(blue_Stringa);                                    //("$" + cmd + "*" + checkSum1);              //Stringa alla Bluetooth
  //gatt.setChar(htsMeasureCharId, temp_measurement, 5);
  //gatt.setChar(htsMeasureCharId, dati_blue);
  //uint32_t temperatura = ((uint32_t)temp_measurement[4] << 24) | ((uint32_t)temp_measurement[3] << 16) | ((uint32_t)temp_measurement[2] << 8) | ((uint32_t)temp_measurement[1]);
  //Serial.println(temperatura);
 
 /*
  ble.print( F("AT+GATTCHAR=") );
  ble.print( hrmMeasureCharId2 );
  ble.print( F(",00-") );
 uint32_t temperatura = ((uint32_t)temp_measurement[4] << 24) | ((uint32_t)temp_measurement[3] << 16) | ((uint32_t)temp_measurement[2] << 8) | ((uint32_t)temp_measurement[1]);

 ble.print(temp_measurement[1], HEX);
 ble.print( F("-") );
 ble.print(temp_measurement[2], HEX);
 ble.print( F("-") );
 ble.print(temp_measurement[3], HEX);
 ble.print( F("-") );
 ble.println(temp_measurement[4], HEX);
 Serial.println(temperatura);
 Serial.println((uint32_t)temp_measurement[4]);
 Serial.print(temp_measurement[1], DEC);
 Serial.print(temp_measurement[2], DEC);
 Serial.print(temp_measurement[3], DEC);
 Serial.println(temp_measurement[4], DEC);*/
  //ble.println(temperatura, HEX);
  
  /* Check if command executed OK */
  if ( !ble.waitForOK() )
  {
    Serial.println(F("Failed to get response!"));
  }  
  }

  float Periodo_beep = min((Vario * 100), 300);
  Periodo_beep = map(Periodo_beep, 0, 300, 740, 100);
  Tono_beep = min((Vario * 100), 500);
  Tono_beep = map(Tono_beep, 0, 500, 2750, 3150);
  Durata_beep =  min((Vario * 100), 500);
  Durata_beep = map(Durata_beep, 0, 500, 380, 50);
  Tono_beep_disc = max((Vario * 100), -500);
  Tono_beep_disc = map(Tono_beep_disc, 0, -500, 2300, 2050);

  if (Vario >= 0.2)
 {
  if( realTime() >= Periodo_beep/1000)
    {
      tone(6, Tono_beep, Durata_beep);
  resetStartTC3();  // reset so never gets above 10 s
      stopTC3();
    //delay(1000); //wait 3 sec
    startTC3();
    } 
 }
 
 if (Vario <= -1.8)
 {
  if( realTime() >= 0.1)
    {
      tone(6, Tono_beep_disc, 200);
  resetStartTC3();  // reset so never gets above 10 s
      stopTC3();
    //delay(1000); //wait 3 sec
    startTC3();
    } 
  
 }
 
}




void TC3_Handler()  // Interrupt on overflow
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
    realTcount++;                    // Increment the supertick register
    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the ovf flag
//  }
}

/*
Get the real time in seconds.
*/
double realTime() 
{   
double  realTime =  (realTcount * 1.2288E-2) + (REG_TC3_COUNT16_COUNT * 1.875E-7) ;;
return realTime;
}


/*
  Setup the Generic clock register
*/


void setTC3clock()
{
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 ));
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_NFRQ; // Set TC as normal Normal Frq
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8;   // Set perscaler
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  // Interrupts
  TC->INTENSET.reg = 0;              // disable all interrupts
  TC->INTENSET.bit.OVF = 1;          // enable overfollow interrup
  // Enable InterruptVector
  NVIC_EnableIRQ(TC3_IRQn);
  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}



void resetStartTC3()
{
   TcCount16* TC = (TcCount16*) TC3; // get timer struct
   realTcount = 0x00;                // Zero superclicks
    TC->CTRLBSET.reg |= TC_CTRLBCLR_CMD_RETRIGGER;   // restart
}

void stopTC3()
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
    TC->CTRLBSET.reg |= TC_CTRLBSET_CMD_STOP;   // Stop counter
}


void startTC3()
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
    TC->CTRLBSET.reg |= TC_CTRLBSET_CMD_RETRIGGER;   //  Start
}

int checkSum(String theseChars) {
  char check = 0;
  // iterate over the string, XOR each byte with the total sum:
    for (int c = 0; c < theseChars.length(); c++) {
    check = int(check ^ theseChars.charAt(c));
  
  } 
  // return the result
    return check;
}


void receiveEvent(int howMany) {
    int c = Wire.read(); // receive byte as a character
   // Serial.println(c);
    if (c <= 8){
      request = c;
    }
  //  Serial.println(request);         // print the character
     
if (c > 8){
    
  int i = 0;
   while (0 < Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
  //  Serial.print(c);         // print the character
    //NMEA_vario += (c);
    dati8[i] = c;
    i++;
           }//Serial.println(dati8);
           String NMEA_POV_1 = dati8;
NMEA_POV_1.trim();
//Serial.println(NMEA_POV_1);
int x = NMEA_POV_1.indexOf(",");
NMEA_POV_1.remove(x);
Vario_al_secondo = NMEA_POV_1;


String NMEA_POV_2 = dati8;
NMEA_POV_2.trim();
x = NMEA_POV_2.indexOf(",");
NMEA_POV_2.remove(0, (x+1)) ;
x = NMEA_POV_2.indexOf(",");
NMEA_POV_2.remove(x);
String Media_P = NMEA_POV_2;


String NMEA_POV_3 = dati8;
NMEA_POV_3.trim();
x = NMEA_POV_3.indexOf(",");
NMEA_POV_3.remove(0, (x+1)) ;
x = NMEA_POV_3.indexOf(",");
NMEA_POV_3.remove(0, (x+1)) ;
String Temp = NMEA_POV_3; 
Vario = Vario_al_secondo.toInt();
Vario = Vario/10;
 //Serial.println(NMEA_RMC);
   String cmd = "POV,E," + String(Vario) + ",P," + Media_P + ",T," + Temp;   // calcolo stringa NMEA OpenVario
  String checkSum1 = String(checkSum(cmd), HEX);
  
 Serial.println("$" + cmd + "*" + checkSum1);              //Stringa alla seriale
 String stringa_nmea_GGA = GPS.stringa_GGA; 
    String stringa_nmea_RMC = GPS.stringa_RMC;
 Serial.print(stringa_nmea_GGA);
 Serial.println(stringa_nmea_RMC);
 //String blue_Stringa = ("$" + cmd + "*" + checkSum1);
 String blue_Stringa = ("$" + cmd + "*" + checkSum1);           //("$POV,E,-0.3,P,934.98,T,28.82*37");  // ("-0.3,934.98,28.82");
 //Serial.print(String(GPS.fix) + "\n");
  float temp_1 = Media_P.toFloat()*100;
  uint32_t temp = uint32_t(temp_1);
  //if (temp<0) {temp =0;}
//Serial.println(temp);
 uint8_t temp_measurement [5] = { 0 };
 temp_measurement [4] = { temp>>24 };
 temp_measurement [3] = { temp>>16 & 0xFF };
 temp_measurement [2] = { temp>>8 & 0xFF };
 temp_measurement [1] = { temp & 0xFF };
  //float2IEEE11073(temp, temp_measurement+1);
  int n_nmea_blue = blue_Stringa.length();
  char dati_blue[n_nmea_blue+1];
//blue_Stringa.toCharArray(dati_blue, n_nmea_blue+1);
ble.print("AT+BLEUARTTX=");
ble.println(blue_Stringa);                                    //("$" + cmd + "*" + checkSum1);              //Stringa alla Bluetooth
}
   // Serial.println("Vario");
//Serial.println(NMEA_vario);
//Serial.println(dati8);
     
    
  }
void requestEvent() {
  
String NMEA;
if (GPS.fix == 1){
NMEA = GPS.stringa_GGA;
}
else
NMEA = "$GPRMC,081711.000,A,3754.2057,N,01325.6122,E,0.00,82.35,220716,,,A*52";
NMEA.trim();
               //$GPRMC,081711.000,A,3754.2057,N,01325.6122,E,5.81,82.35,220716,,,A*52
int n_nmea = NMEA.length();               
String NMEA1 = NMEA.substring(0, 24);
String NMEA2 = NMEA.substring(24, 48);
String NMEA3 = NMEA.substring(48, n_nmea);
//Serial.println(n_nmea);
//Serial.println(NMEA1);
//Serial.println(NMEA2);
//Serial.println(NMEA3);

char dati[25];
NMEA1.toCharArray(dati, 25);
//Serial.println(dati);
char dati2[25];
NMEA2.toCharArray(dati2, 25);


char dati3[(n_nmea - 47)];
NMEA3.toCharArray(dati3, (n_nmea - 47));

int n = strlen(dati3);
//Serial.print("Lunghezza stringa");
//Serial.println(n);


String NMEA4;
if (GPS.fix == 1){
NMEA4 = GPS.stringa_RMC;
}
else
NMEA4 = "$GPRMC,081711.000,A,3754.2057,N,01325.6122,E,5.81,82.35,220716,,,A*52";
NMEA4.trim();
               //$GPRMC,081711.000,A,3754.2057,N,01325.6122,E,5.81,82.35,220716,,,A*52
int n_nmea1 = NMEA4.length();               
String NMEA5 = NMEA4.substring(0, 24);
String NMEA6 = NMEA4.substring(24, 48);
String NMEA7 = NMEA4.substring(48, n_nmea1);
//Serial.println(n_nmea1);
//Serial.println(NMEA5);
//Serial.println(NMEA6);
//Serial.println(NMEA7);

char dati4[25];
NMEA5.toCharArray(dati4, 25);
char dati5[25];
NMEA6.toCharArray(dati5, 25);


char dati6[(n_nmea1 - 47)];
NMEA7.toCharArray(dati6, (n_nmea1 - 47));

int n2 = strlen(dati6);
//Serial.print("Lunghezza stringa2");
//Serial.println(n2);





switch(request)
{
  case 0:
if (GPS.fix == 0){ 
Wire.write(0);
}
else
Wire.write(1);
break;
case 1:
//Serial.println("requestuno" + String(request) + "boo"); 
Wire.write(dati); // respond with message of 6 bytes//istruzioni
break;
case 2:
//Serial.println("request2" + String(request) + "boo");
Wire.write(dati2); // respond with message of 6 bytes//istruzioni
break;
case 3:
Wire.write(dati3); // respond with message of 6 bytes//istruzioni
break;
case 4:
Wire.write(n); // respond with message of 6 bytes//istruzioni
break;
case 5:
Wire.write(dati4); // respond with message of 6 bytes//istruzioni
break;
case 6:
Wire.write(dati5); // respond with message of 6 bytes//istruzioni
break;
case 7:
Wire.write(dati6); // respond with message of 6 bytes//istruzioni
break;
case 8:
Wire.write(n2); // respond with message of 6 bytes//istruzioni
break;
}

}



