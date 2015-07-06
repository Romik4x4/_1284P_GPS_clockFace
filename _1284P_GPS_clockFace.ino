//
// Arduino 1.0.1
//

#include <Wire.h>
#include <Time.h>                         // download from: http://www.arduino.cc/playground/Code/Time
#include <glcd.h>                         // Graphics LCD library   
#include "fonts/fixednums7x15.h"          // system font
#include "fonts/TimeFont.h"               // system font
#include "fonts/fixednums15x31.h"
#include "fonts/SystemFont5x7.h"

#include <TinyGPS.h>
#include <Sunrise.h>

#include <SoftwareSerial.h>

SoftwareSerial gps_out(7,6); // RX, TX GPS Out vi Bluetooth HC-05 Speed 4800

#define GPS_STAT 1

#define DS1307_ADDRESS 0x68

// ------------------- GLOBAL DATE TIME POSITION -------------------------------------

  int Up_m = 0;
  int Up_h = 0;
  int D_m = 0;
  int D_h = 0;
  
  int g_year;
  byte g_month, g_day, g_hour, g_minutes, g_second, g_hundredths;
  unsigned long g_age;
  unsigned long g_fix_age;
  float g_lat, g_lon;

// -----------------------------------------------------------------------------------

TinyGPS gps;

// -----------------------------------------------------------------------------------

long previousMillis = 0; 
long interval = 1000; 

// ------------------------- BMP085 ---------------------------------------

#define BMP085_ADDRESS 0x77   // I2C address of BMP085

const unsigned char OSS = 0;  // Oversampling Setting

int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
long b5;

// ---------------------- END BMP085 -----------------------------------------

unsigned long currentMillis;

int radius = 1;
int color = BLACK;
char c;
int angle = 0;
 
// ------------------------------ Setup -------------------------------------

void setup() {

  Wire.begin();
  
  bmp085Calibration();

  GLCD.Init();                // start the GLCD code
  GLCD.SelectFont(TimeFont);  // romik0.h

  setTime(23,37,0,2,1,10);    // set time to 4:37 am Jan 2 2010  

  pinMode(GPS_STAT,OUTPUT);   // LED GPS Status
  digitalWrite(GPS_STAT,LOW);
  
  Serial1.begin(4800);        // GPS Connected Serial One port
  
  Serial1.print("$PSRF100,1,4800,8,1,0*0E\r\n");
  
  gps_out.begin(4800);
  
  // setDateTime();
 
}

// ----------------------------- Loop --------------------------------------

void  loop() {  

  currentMillis = millis();
  
  if (Serial1.available()) {  
   c = Serial1.read();
   gps_out.print(c);
   if (gps.encode(c)) {
    set_gps_values();
    digitalWrite(GPS_STAT,HIGH);
   } 
  }
  
  if(currentMillis - previousMillis > interval) {
   previousMillis = currentMillis; 
   g_print_time();      
   Sun();
   bmp();
  }   
  
}

// ------------------------------------------------ Functions ------------------------------------------

void g_print_time( void ) {
    
 // ------------ Печатаем время ------------------------------------
 
 GLCD.CursorToXY(0,0);

  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(DS1307_ADDRESS, 7);

  int s = bcdToDec(Wire.read());
  int m = bcdToDec(Wire.read());
  int h = bcdToDec(Wire.read() & 0b111111); //24 hour time
  
  int weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
  int monthDay = bcdToDec(Wire.read());
  int month = bcdToDec(Wire.read());
  int year = bcdToDec(Wire.read());

   // byte h = hour();
   // byte m = minute();
   // byte s = second();

 GLCD.SelectFont(TimeFont);
 
 if (h < 10) GLCD.print("0"); GLCD.print(h,DEC); 
 GLCD.print(":");
 if (m < 10) GLCD.print("0"); GLCD.print(m,DEC); 
 GLCD.print(":");
 if (s < 10) GLCD.print("0"); GLCD.print(s,DEC); 

// ----------- Печатать Месяц и день -----------------------------
 
 GLCD.CursorToXY(63,26);
 GLCD.SelectFont(System5x7);
    
 if (monthDay < 10) GLCD.print("0");   GLCD.print(monthDay,DEC);
 GLCD.print("-");
 if (month < 10) GLCD.print("0"); GLCD.print(month,DEC);    
 
}

// ---------------- Sun Rise and Sun Set Выводим на экран -------------------------------

void Sun( void ) { 
  
    GLCD.SelectFont(fixednums7x15);  // Select Font
    
    GLCD.CursorToXY(0,27);
    if (Up_h<10) GLCD.print("0"); GLCD.print(Up_h,DEC);     
    GLCD.print(":");     
    if (Up_m<10) GLCD.print("0"); GLCD.print(Up_m,DEC);     
  
    GLCD.CursorToXY(0,45);
    if (D_h<10) GLCD.print("0"); GLCD.print(D_h,DEC);     
    GLCD.print(":");     
    if (D_m<10) GLCD.print("0"); GLCD.print(D_m,DEC);     
    
    int x = 50;
    int y = 33;
    int r_radius = 7;
    
    GLCD.FillCircle(x,y,r_radius,WHITE);
    GLCD.DrawCircle(x,y,r_radius); 
    
    float rad = (angle/1) * (3.14/150);
    
    int x1 = x + r_radius * cos(rad);
    int y1 = y + r_radius * sin(rad);

    angle+=10; if (angle > 360 ) angle = 0;
    
    GLCD.DrawLine(x,y,x1,y1,BLACK);
        
    GLCD.DrawCircle(50,52,7); for(int i=1;i<=radius;i++) GLCD.DrawCircle(50,52,i,color);  
    
    radius++;
    
    if ( radius > 7 ) { radius = 1; if (color == BLACK) color = WHITE; else color = BLACK; }
   
}


// ------------------------------ Устанавливаем данные со спутника ------------------------------------

void set_gps_values( void ) {
  
  int t;
  byte _hour;
  
  gps.crack_datetime(&g_year, &g_month, &g_day, &g_hour, &g_minutes, &g_second, &g_hundredths, &g_age);
 
  if (g_age != TinyGPS::GPS_INVALID_AGE) {
      
  gps.f_get_position(&g_lat, &g_lon, &g_fix_age);

  _hour = g_hour + 4;

  if (_hour > 24) { 
   g_hour = g_hour; 
  } else { 
   g_hour = _hour; 
  }
  
  if (_hour == 24) { 
   g_hour = 0; 
  }

  setTime(g_hour,g_minutes,g_second,g_day,g_month,g_year); // set time to 4:37 am Jan 2 2010 
  
  // ============== Set Date for DS1307 ======================
  
  byte weekDay =     2;  //1-7

  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0); //stop Oscillator

  Wire.write(decToBcd(g_second));
  Wire.write(decToBcd(g_minutes));
  Wire.write(decToBcd(g_hour));
  Wire.write(decToBcd(weekDay));
  Wire.write(decToBcd(g_day));
  Wire.write(decToBcd(g_month));
  Wire.write(decToBcd(g_year));

  Wire.write(0); //start
  Wire.endTransmission();

  // =========================================================
    
  Sunrise mySunrise(g_lat,g_lon,4);

  mySunrise.Actual();

  t = mySunrise.Rise(g_month,g_day);
     
  if(t >= 0) {
    Up_h = mySunrise.Hour();
    Up_m = mySunrise.Minute();
  }
  
  t = mySunrise.Set(g_month,g_day);
  
  if(t >= 0) {
    D_h = mySunrise.Hour();
    D_m = mySunrise.Minute();
  }

 }
  
}


// ------------------------- BMP085 Functions ---------------------------------

// --------------- Stores all of the bmp085's calibration values into global variables
// --------------- Calibration values are required to calculate temp and pressure
// --------------- This function should be called at the beginning of the program

void bmp085Calibration( void ) {
  
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
  
}

// ----------------------------- Calculate temperature in deg C ----------------------------------------

float bmp085GetTemperature(unsigned int ut) {

  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// ------------------ Calculate pressure given up
// ------------------ calibration values must be known
// ------------------ b5 is also required so bmp085GetTemperature(...) must be called first.
// ------------------ Value returned will be pressure in units of Pa.

long bmp085GetPressure(unsigned long up) {
  
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  
  // ------ Calculate B3
  
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // ----- Calculate B4

  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// ------------------------------------- Read 1 byte from the BMP085 at 'address' --------------------------------

char bmp085Read(unsigned char address) {
  
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available());

  return Wire.read();
}

// --------------------------------------- Read 2 bytes from the BMP085
// --------------------------------------- First byte will be from 'address'
// --------------------------------------- Second byte will be from 'address'+1

int bmp085ReadInt(unsigned char address) {
  
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2);
    
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// -------------------------------- Read the uncompensated temperature value ---------------------------------------

unsigned int bmp085ReadUT(){

  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms

  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7

  ut = bmp085ReadInt(0xF6);
  
  return ut;
}

// ------------------------------ Read the uncompensated pressure value ------------------------------------------

unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  
  msb  = bmp085Read(0xF6);
  lsb  = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

// ---------------------------------------- Запись регистров -----------------------------

void writeRegister(int deviceAddress, byte address, byte val) {
  
  Wire.beginTransmission(deviceAddress);  // start transmission to device 
  Wire.write(address);                    // send register address
  Wire.write(val);                        // send value to write
  Wire.endTransmission();                 // end transmission
}


// ------------------ Чтение регистров --------------------------------------------------

int readRegister(int deviceAddress, byte address){

  int v;
  
  Wire.beginTransmission(deviceAddress);
  Wire.write(address);                    // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1);     // read a byte

  while(!Wire.available());

  v = Wire.read();
  return v;
}

// -------------------------------------- Высота -------------------------------------

float calcAltitude(float pressure) {

  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;

  return C;
}

// ------------------------- Печать BMP Дата ------------------------------------------

void bmp( void ) {

    float temperature = bmp085GetTemperature(bmp085ReadUT()); // MUST be called first
    float pressure = bmp085GetPressure(bmp085ReadUP());
    float atm = pressure / 101325;                            // standard atmosphere
    float altitude = calcAltitude(pressure);                  // Uncompensated caculation - in Meters 
    
    GLCD.SelectFont(System5x7);

    GLCD.CursorToXY(63,36);
    GLCD.print(temperature);
    GLCD.print("C");
    
    GLCD.CursorToXY(63,46);
    GLCD.print(pressure/133.322,0);
    GLCD.print(" N");
    GLCD.print(g_lat);
    
    GLCD.CursorToXY(63,56);
    GLCD.print(altitude,0);
    GLCD.print(" E");
    GLCD.print(g_lon);
    
    GLCD.SelectFont(fixednums7x15);
    
    GLCD.CursorToXY(100,0);
    
    if ( minute() > 30 && minute() < 35) GLCD.print(pressure/133.322,0);
    
    GLCD.CursorToXY(100,20);

    if ( minute() > 0 && minute() < 5) GLCD.print(pressure/133.322,0);

}

// ========================= DS1307 =============================================

void setDateTime() {

  byte second =      00; //0-59
  byte minute =      14; //0-59
  byte hour =        14; //0-23
  byte weekDay =     4;  //1-7
  byte monthDay =    31; //1-31
  byte month =       10;  //1-12
  byte year  =       13; //0-99

  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0); //stop Oscillator

  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write(decToBcd(weekDay));
  Wire.write(decToBcd(monthDay));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));

  Wire.write(0); //start
  Wire.endTransmission();

}

byte decToBcd(byte val) {
  return ( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val) {
  return ( (val/16*10) + (val%16) );
}

