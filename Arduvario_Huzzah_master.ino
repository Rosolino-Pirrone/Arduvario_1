#include <SoftwareSerial.h>
SoftwareSerial serial_2 (0, 2);


#include <Wire.h>
#include <math.h>
#include <STRING.h>


#define MS5611_ADDRESS (0x77)
#define CMD_RESET (0x1E) 
#define CMD_ADC_D1_4096 (0x48)
#define CMD_ADC_D2_4096 (0x58)
#define CMD_ADC_READ (0x00)
#define CMD_PROM_RD_1 (0xA2)
#define CMD_PROM_RD_2 (0xA4)
#define CMD_PROM_RD_3 (0xA6)
#define CMD_PROM_RD_4 (0xA8)
#define CMD_PROM_RD_5 (0xAA)
#define CMD_PROM_RD_6 (0xAC)

uint16_t C_1;
uint16_t C_2;
uint16_t C_3;
uint16_t C_4;
uint16_t C_5;
uint16_t C_6;

uint32_t D_1;
uint32_t D_2;

int64_t dt;
float TEMP, T_2;

int64_t OFF_1, OFF_2;
int64_t SENS, SENS_2;

float P;

float Valori[2];
float Media_valori_alt[50];
float Old_media_altitudine;

unsigned long previousMillis = 0;
unsigned long previousMillis_velocita = 0;
const long interval = 1000;
int previous_time = 180;
float Tono_beep = 0;
float Durata_beep;
float Tono_beep_disc;
float Media_P;
float somma;
String oldROS;

void setup() {                                 // put your setup code here, to run once:

Serial.begin(115200);                          // inizializzo la seriale a questa velocità

serial_2.begin(9600);

  delay(5000);                                  // attendo 5s

  Wire.begin();                                // inizializzo la i2c senza nessun indirizzo, imposto Arduino come master
  Wire.setClockStretchLimit(1500);
  Wire.beginTransmission(MS5611_ADDRESS);      // comunico tramite i2c all'indirizzo del sensore
  Wire.write(CMD_RESET);                       // impartisco uno reset
  Wire.endTransmission();                      // fine comunicazione
  delay (3);                                   // attendo 3 ms
  
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_PROM_RD_1);                   // comando per leggere la prom c1
  Wire.endTransmission();
  //delay (10);
  Wire.requestFrom(MS5611_ADDRESS, 2);         // chiedo la lettura del valore di due byte
  while(Wire.available())                      // attendo che sia pronto il valore restituito
    {
    uint8_t B_H = Wire.read();                 // leggo il primo byte alto
    uint8_t B_L = Wire.read();                 // leggo il secondo byte basso
    C_1 = ((uint16_t)B_H << 8) | B_L;          // unisco i due byte per formare un int di 16 bit
    }

//Serial.print(" C_1: ");                     // stampo a monitor seriale
//Serial.println(C_1);   

Wire.beginTransmission(MS5611_ADDRESS);
Wire.write(CMD_PROM_RD_2);
Wire.endTransmission();
//delay (10);

Wire.requestFrom(MS5611_ADDRESS, 2);
while(Wire.available())
    {
    uint8_t B_H = Wire.read(); 
    uint8_t B_L = Wire.read();
    C_2 = ((uint16_t)B_H << 8) | B_L;
    }

//Serial.print(" C_2: ");
//Serial.println(C_2); 

Wire.beginTransmission(MS5611_ADDRESS);
Wire.write(CMD_PROM_RD_3);
Wire.endTransmission();
//delay (10);
Wire.requestFrom(MS5611_ADDRESS, 2);
while(Wire.available())
    {
    uint8_t B_H = Wire.read(); 
    uint8_t B_L = Wire.read();
C_3 = ((uint16_t)B_H << 8) | B_L;
    }

//Serial.print(" C_3: ");
//Serial.println(C_3); 

Wire.beginTransmission(MS5611_ADDRESS);
Wire.write(CMD_PROM_RD_4);
Wire.endTransmission();
//delay (10);
Wire.requestFrom(MS5611_ADDRESS, 2);
while(Wire.available())
    {
    uint8_t B_H = Wire.read(); 
    uint8_t B_L = Wire.read();
    C_4 = ((uint16_t)B_H << 8) | B_L;
    }

//Serial.print(" C_4: ");
//Serial.println(C_4); 

Wire.beginTransmission(MS5611_ADDRESS);
Wire.write(CMD_PROM_RD_5);
Wire.endTransmission();
//delay (10);

Wire.requestFrom(MS5611_ADDRESS, 2);
while(Wire.available())
    {
    uint8_t B_H = Wire.read(); 
    uint8_t B_L = Wire.read();
    C_5 = ((uint16_t)B_H << 8) | B_L;
    }

//Serial.print(" C_5: ");
//Serial.println(C_5); 

Wire.beginTransmission(MS5611_ADDRESS);
Wire.write(CMD_PROM_RD_6);
Wire.endTransmission();
//delay (10);
Wire.requestFrom(MS5611_ADDRESS, 2);
while(Wire.available())
    {
    uint8_t B_H = Wire.read(); 
    uint8_t B_L = Wire.read();
    C_6 = ((uint16_t)B_H << 8) | B_L;
    }

//Serial.print(" C_6: ");
//Serial.println(C_6);

//Timer1.initialize(1000000);
}




void loop() {                           // put your main code here, to run repeatedly:

 unsigned long currentMillis = millis();
 /*int time = 0;
  if (currentMillis - previousMillis > interval) 
  {
    time = previous_time -1;
    
  }
 
 if (currentMillis - previousMillis < interval) 
  {
    time = previous_time + 1;
  }
  previous_time = time;
  */
 unsigned long Tempo = currentMillis - previousMillis_velocita;
 previousMillis_velocita = currentMillis;
 Serial.println(Tempo);
 

 
 for (uint8_t i = 0; i < 15; i++)        // ciclo for per memorizzare 10 rilevazioni in un secondo
 {
  Valori_Alt_Temp();                      // richiamo la funzione Valori_Alt_Temp
  Media_valori_alt[i] = Valori[1];        // memorizzo i valori nell'array Media_Valori_Alt
 serial_2.println(oldROS);
  //delay (time);                            // pausa di 78 millisecondi
  
 }

 // somma dei valori e calcolo della media
 for (uint8_t i = 0; i < 15; i++) 
 {
  somma = somma + Media_valori_alt[i];
 }
 Media_P = somma / 15;
 //Serial.println("Media" + String(Media_P));
  float altitudine = 44330.0 * (1.0 - pow(Media_P / 1013.25, 0.1903));
  somma = 0;
  float Vario = altitudine - Old_media_altitudine;    // sottrazione quota calcolo vario in m/s
  Old_media_altitudine = altitudine;                 // media precedente
  if (Vario > 50)
  {
   Vario = 0; 
  }
float Vario_al_secondo = Vario / Tempo * 1000;
  
  ///////////////////if (currentMillis - previousMillis >= interval) 
 // {previousMillis = currentMillis;
  
  String cmd = "POV,E," + String(Vario_al_secondo).substring(0,4) + ",P," + String(Media_P) +",T," + String(Valori[0] / 100);   // calcolo stringa NMEA OpenVario
  String checkSum0 = String(checkSum(cmd), HEX);
  String str_to_m0 = String(Vario_al_secondo*10).substring(0,4) + "," + String(Media_P) +"," + String(Valori[0] / 100);
  Serial.println("$" + cmd + "*" + checkSum0);              //Stringa alla seriale
  //Serial.println(str_to_m0); 
int n_str_to_m0 = str_to_m0.length();
char dati_8[(n_str_to_m0 + 1)];
str_to_m0.toCharArray(dati_8, (n_str_to_m0 + 1));
//Serial.println(dati_8);

Wire.beginTransmission(8);
Wire.write(9);
Wire.write(dati_8);
Wire.endTransmission(8);

delay(5);
int fix;
  Wire.beginTransmission(8);
  Wire.write(0);
  Wire.endTransmission(8);
  //delay(5);
  Wire.requestFrom(8, 1);
  while (Wire.available()) 
  {
    fix = Wire.read(); // receive a byte as int
  }
//Serial.println(String(fix));


int i = 0;


  char dati[24];
  char dati2[24];
  
  String stringa;
  String stringa2;
  String stringa3;
  stringa.reserve(48);
  stringa2.reserve(48);
  stringa3.reserve(48);
if (fix ==1)
{
  
  Wire.beginTransmission(8);
  Wire.write(1);
  Wire.endTransmission(8);
  //delay(5);
  Wire.requestFrom(8, 24);    // request 6 bytes from slave device #8
  i = 0;
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    //Serial.print(c);         // print the character
    stringa += (c);
    dati[i] = c;
    i++;
  }
  //stringa += ("boo");
  //Serial.println(stringa);

  Wire.beginTransmission(8);
  Wire.write(2);
  Wire.endTransmission(8);
  //delay(5);
  Wire.requestFrom(8, 24);    // request 6 bytes from slave device #8
  i = 0;
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    //Serial.print(c);         // print the character
    stringa += (c);
    dati2[i] = c;
    i++;
  }
//Serial.print(stringa2);
  int n = 0;


  Wire.beginTransmission(8);
  Wire.write(4);
  Wire.endTransmission(8);
  //delay(5);
  Wire.requestFrom(8, 1);    // request 6 bytes from slave device #8
  while (Wire.available()) { // slave may send less than requested
    n = Wire.read(); // receive a byte as character
    //Serial.print(c);         // print the character
    
  }
  //Serial.print("variabile");
  //Serial.println(n);

  
char dati3[n];
Wire.beginTransmission(8);
  Wire.write(3);
  Wire.endTransmission(8);
  //delay(5);
  Wire.requestFrom(8, n);    // request 6 bytes from slave device #8
  i = 0;
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    //Serial.print(c);         // print the character
    stringa += (c);
    dati3[i] = c;
    i++;
  }
  //String stringa3 = dati3;
 //Serial.println(stringa3);

  Serial.println(stringa);
  
}





char dati4[24];
  char dati5[24];
  
  String stringa4;
  String stringa5;
  String stringa6;
  stringa4.reserve(48);
  stringa5.reserve(48);
  stringa6.reserve(48);
if (fix ==1)
{  
  Wire.beginTransmission(8);
  Wire.write(5);
  Wire.endTransmission(8);
  //delay(5);
  Wire.requestFrom(8, 24);    // request 6 bytes from slave device #8
  i = 0;
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    //Serial.print(c);         // print the character
    stringa4 += (c);
    dati4[i] = c;
    i++;
  }
  //Serial.print(stringa4);

  Wire.beginTransmission(8);
  Wire.write(6);
  Wire.endTransmission(8);
  //delay(5);
  Wire.requestFrom(8, 24);    // request 6 bytes from slave device #8
  i = 0;
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    //Serial.print(c);         // print the character
    stringa4 += (c);
    dati5[i] = c;
    i++;
  }
//Serial.print(stringa5);
  int n2 = 0;


  Wire.beginTransmission(8);
  Wire.write(8);
  Wire.endTransmission(8);
  //delay(5);
  Wire.requestFrom(8, 1);    // request 6 bytes from slave device #8
  while (Wire.available()) { // slave may send less than requested
    n2 = Wire.read(); // receive a byte as character
    //Serial.print(c);         // print the character
    
  }
  //Serial.print("variabile");
  //Serial.println(n2);

  
char dati6[n2];
Wire.beginTransmission(8);
  Wire.write(7);
  Wire.endTransmission(8);
  //delay(5);
  Wire.requestFrom(8, n2);    // request 6 bytes from slave device #8
  i = 0;
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    //Serial.print(c);         // print the character
    stringa4 += (c);
    dati6[i] = c;
    i++;
  }
  //String stringa6 = dati6;
 //Serial.println(stringa3);

  Serial.println(stringa4);     //RMC
}

int q;
for (i=0; i<7; i++)
{ 
q = stringa4.indexOf(",");
stringa4.remove(0, (q+1)) ;
}
q = stringa4.indexOf(",");
stringa4.remove(q);
int altitudine1 =altitudine;
float velo;
/*
if (!stringa4.toInt())
{
  velo="0";
}
else*/
/*
velo = String(stringa4.toInt()*1,852);
if (!velo){
  velo ="0";
}
else*/
float eff;
velo = (stringa4.toFloat()*1.852*10);                      // questi calcoli potranno sembrare errati, servono per la visualizzazione 
int velo2 = int(velo);
if (Vario_al_secondo >= 0){
  eff = 999;                                                         //nel display che accetta solo interi
}
else

  eff = min(velo2/(Vario_al_secondo*3.6*(-1)), 999);
int eff2 = int(eff);
String ROS = "$ROS," + String(Vario_al_secondo).substring(0,4) + "," + String(altitudine1) + "," + String(Valori[0] / 100) + ","  + String(velo2) + "," + String(fix) + "," + String(eff2) +"," +",";
serial_2.println(ROS);     //$ROS
//Serial.println(ROS);     //$ROS
//Serial.println("velo" + String(velo));

oldROS=ROS;

/*
String NMEA_RMC = stringa4 + stringa5 + stringa6;
for (int i =0; i < 13; i++){
  int x = NMEA_RMC.indexOf(",");
  NMEA_RMC.remove(0, (x+1)) ;
 
  
  i++;
}
int x = NMEA_RMC.indexOf(",");
 NMEA_RMC.remove(x);
 //Serial.println(NMEA_RMC);
*/

  
 /*
   
  float Periodo_beep = min((Vario_al_secondo * 100), 500);
  Periodo_beep = map(Periodo_beep, 0, 500, 670000, 100000);
  Tono_beep = min((Vario_al_secondo * 100), 500);
  Tono_beep = map(Tono_beep, 0, 500, 600, 850);
  Durata_beep =  min((Vario_al_secondo * 100), 500);
  Durata_beep = map(Durata_beep, 0, 500, 500, 50);
  Tono_beep_disc = max((Vario_al_secondo * 100), -500);
  Tono_beep_disc = map(Tono_beep_disc, 0, -1000, 250, 235);

  if (Vario >= 0.1)
 {
  Timer1.setPeriod(Periodo_beep);
  //Timer1.restart();
  Timer1.attachInterrupt(suona);
   
 }
 if (Vario <= -0.2)
 {
  Timer1.setPeriod(1000);
  //Timer1.restart();
  Timer1.attachInterrupt(suona_disc);
  
 }
 if (Vario < 0.1 && Vario > -0.2)
 {
  Timer1.stop();
 }
 */
 
 

/////////////////}  
}

// funzione rilevazione D1
uint32_t ghet_d_1()
{    
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(CMD_ADC_D1_4096);
    Wire.endTransmission();
    delay (10);
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(CMD_ADC_READ);
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDRESS, 3);
    while(Wire.available())
    {
    uint8_t B_H = Wire.read(); 
    uint8_t B_M = Wire.read();
    uint8_t B_L = Wire.read(); 
    D_1 = ((int32_t)B_H << 16) | ((int32_t)B_M << 8) | B_L;
    }
    return D_1;
}

// funzione rilevazione D2
uint32_t ghet_d_2()
{    
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(CMD_ADC_D2_4096);
    Wire.endTransmission();
    delay (10);
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(CMD_ADC_READ);
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDRESS, 3);
    while(Wire.available())
    {
    uint8_t B_H = Wire.read(); 
    uint8_t B_M = Wire.read();
    uint8_t B_L = Wire.read(); 
    D_2 = ((int32_t)B_H << 16) | ((int32_t)B_M << 8) | B_L;
    }
    return D_2;
}

// funzione calcolo Temperatura e Quota memorizzati in un array di 2
float Valori_Alt_Temp()
{
    D_1 = ghet_d_1();
    D_2 = ghet_d_2();

    dt = D_2 - C_5 * pow(2, 8);
    TEMP = 2000 + dt * C_6 / pow(2, 23);
    OFF_1 = C_2 * pow(2, 16) + C_4 * dt / pow(2, 7);
    SENS = C_1 * pow(2, 15) + C_3 * dt / pow(2, 8);
    T_2 = 0;
    OFF_2 = 0;

    if (TEMP < 2000) 
  {
    T_2 = pow(dt, 2) / pow(2, 31);
    OFF_2 = 5 * pow((TEMP - 2000), 2) / 2;
    SENS_2 = 5 * pow((TEMP - 2000), 2) / pow(2, 2); 
  }

if (TEMP < -1500)
  {
    OFF_2 = OFF_2 + 7 * pow((TEMP + 1500), 2);
    SENS_2 = SENS_2 + 11 * pow((TEMP + 1500), 2)/ 2;
  }

    OFF_1 = OFF_1 - OFF_2;
    SENS = SENS - SENS_2;
    TEMP = TEMP - T_2;

    P = ((D_1 * SENS / pow(2, 21) - OFF_1) / pow(2, 15))  / 100;

    float altitudine = 44330.0 * (1.0 - pow(P / 1013.25, 0.1903)); 


    Valori[0] = TEMP;
    Valori[1] = P;

}

// funzione calcolo checkSum stringa NMEA
int checkSum(String theseChars) {
  char check = 0;
  // iterate over the string, XOR each byte with the total sum:
    for (int c = 0; c < theseChars.length(); c++) {
    check = int(check ^ theseChars.charAt(c));
  
  } 
  // return the result
    return check;
}
/*
void suona()
{
  Timer1.pwm(9, Tono_beep, 500);
  
}

void suona_disc()
{
  tone(9, Tono_beep_disc, 10);
}*/
