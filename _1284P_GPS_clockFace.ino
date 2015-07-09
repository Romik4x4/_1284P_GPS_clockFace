//
// Arduino 1.0.6
//

#include <Wire.h>
#include <glcd.h>                            // Graphics LCD library   
#include "fonts/fixednums7x15.h"    // system font
#include "fonts/TimeFont.h"            // system font
#include "fonts/fixednums15x31.h"
#include "fonts/SystemFont5x7.h"
#include <RTClib.h>
#include <TinyGPS++.h>
#include <Sunrise.h>
#include <SoftwareSerial.h>
#include <Average.h>
#include <BMP085.h>
#include <EEPROM.h>

#define DEBUG 0

// ----------------------- BMP085 ---------------------------------

struct bmp085_data_in // Данные о давлении,высоте и температуре
{    
  double Press;
  unsigned long unix_time; 

} 
bmp085_data;

struct bmp085_data_out  // Данные о давлении,высоте и температуре
{    
  double Press;
  unsigned long unix_time; 

} 
bmp085_out;

Average<double> bar_avr(100);
 
#define TWO_DAYS 172800

#define GPS_STAT  1 // D1 LED

#define UTC 3 //  UTC+3 = Moscow

#define DS1307_ADDRESS 0x68

#define TIMECTL_MAXTICS 4294967295L
#define TIMECTL_INIT          0

RTC_DS1307 rtc;  // DS1307 RTC Real Time Clock

#define FIVE_MINUT 300000

unsigned long SetgpsTimeMark     = 0;
unsigned long SetgpsTimeInterval = 1000*60*10;  // 10 Минут

unsigned long TimeMark = 0;
unsigned long TimeInterval = 800;  // Каждую секунду

unsigned long bmpTimeMark = 0;
unsigned long bmpTimeInterval = FIVE_MINUT;  // Каждую минуту

unsigned long saveTimeMark = 0;
unsigned long saveTimeInterval = FIVE_MINUT*3;  // Каждые 20 минут

unsigned long showTimeMark = 0;
unsigned long showTimeInterval = FIVE_MINUT/2;  // Каждые 2 минуты


// -----------------------------------------------------------------------------------

TinyGPSPlus gps;

BMP085 dps = BMP085();   

long Temperature = 0, Pressure = 0, Altitude = 0;

SoftwareSerial gps_out(7,6); // RX, TX GPS Out vi Bluetooth HC-05 Speed 4800

// -----------------------------------------------------------------------------------

unsigned long currentMillis;

int radius = 1;
int color = BLACK;
char c;
int angle = 0;

// ------------------------------ Setup -------------------------------------

void setup() {

  Wire.begin();

  rtc.begin();

  dps.init(MODE_ULTRA_HIGHRES, 25000, true);  // Разрешение BMP180

  GLCD.Init();                             
  GLCD.SelectFont(TimeFont); 

  pinMode(GPS_STAT,OUTPUT);   // LED GPS Status
  digitalWrite(GPS_STAT,LOW);

  Serial1.begin(4800);        // GPS Connected Serial One port

  Serial1.print("$PSRF100,1,4800,8,1,0*0E\r\n");

  gps_out.begin(4800);

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  Sun();

  // x,y - рисует линейку

  GLCD.DrawLine( 45,60, 127, 60, BLACK);  // Горизонт
  GLCD.DrawLine( 45,26, 45, 60, BLACK);    // Вертикаль

  //  25 |
  //       |
  //       |
  //       |
  //  60 |
  //  ------------------------------------------------------- 127x
  //  45x

  dps.getTemperature(&Temperature);  // Температура
  dps.getPressure(&Pressure);              // Давление
  dps.getAltitude(&Altitude);                   // Высота 

  bmp085_data.Press = Pressure/133.3;

  GLCD.SelectFont(System5x7);
  GLCD.CursorToXY(92,1);
  GLCD.print(bmp085_data.Press);

  GLCD.SelectFont(System5x7);
  GLCD.CursorToXY(92,12);
  GLCD.print(Temperature/10.0);

  Show_Bar_Data();

}

// ----------------------------- Loop --------------------------------------

void  loop() {  

  currentMillis = millis();

  if (isTime(&SetgpsTimeMark,SetgpsTimeInterval))  { // Каждые 10 минут
    set_GPS_DateTime(); 
    Sun();
  }

  if (!DEBUG) {
    if (Serial1.available()) {  
      char c = Serial1.read();
      gps_out.print(c);
    }
  }

  if (isTime(&TimeMark,TimeInterval))  { 

    g_print_time();     

    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
      digitalWrite(GPS_STAT,HIGH);
    } 
    else {
      digitalWrite(GPS_STAT,LOW);
    }

  }   

  if (isTime(&saveTimeMark,saveTimeInterval))  { // Каждые 20 минут
    Save_Bar_Data(); 
  }

  if (isTime(&showTimeMark,showTimeInterval))  { // Каждые 2 минуты
    Show_Bar_Data(); 
  }

  if (isTime(&bmpTimeMark,bmpTimeInterval))  { // Каждые 1 минутa

    dps.getTemperature(&Temperature);  // Температура
    dps.getPressure(&Pressure);              // Давление
    dps.getAltitude(&Altitude);                   // Высота 

    bar_avr.push(Pressure/133.3);     
 
    bmp085_data.Press = bar_avr.mean();

    GLCD.SelectFont(System5x7);
    GLCD.CursorToXY(92,1);
    GLCD.print(bar_avr.mean());

    GLCD.SelectFont(System5x7);
    GLCD.CursorToXY(92,12);
    GLCD.print(Temperature/10.0);

  }

}



// ------------------------------------------------ Functions ------------------------------------------

int isTime(unsigned long *timeMark, unsigned long timeInterval) {

  unsigned long timeCurrent;
  unsigned long timeElapsed;
  int result=false;

  timeCurrent = millis();
  if (timeCurrent < *timeMark) {
    timeElapsed=(TIMECTL_MAXTICS-*timeMark) + timeCurrent;
  } 
  else {
    timeElapsed=timeCurrent-*timeMark;
  }

  if (timeElapsed>=timeInterval) {
    *timeMark=timeCurrent;
    result=true;
  }
  return(result);
}

void set_GPS_DateTime() {

  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {

    DateTime utc = (DateTime (gps.date.year(), 
    gps.date.month(), 
    gps.date.day(),
    gps.time.hour(),
    gps.time.minute(),
    gps.time.second()) + 60 * 60 * UTC);

    rtc.adjust(DateTime(utc.unixtime()));

  }
}

void g_print_time( void ) {

  // ------------ Печатаем время ------------------------------------

  GLCD.CursorToXY(0,0);

  DateTime now = rtc.now();

  GLCD.SelectFont(TimeFont);

  if (now.hour() < 10) GLCD.print("0"); 
  GLCD.print(now.hour(),DEC); 
  GLCD.print(":");
  if (now.minute() < 10) GLCD.print("0"); 
  GLCD.print(now.minute(),DEC); 
  GLCD.print(":");
  if (now.second() < 10) GLCD.print("0"); 
  GLCD.print(now.second(),DEC); 

}

// ---------------- Sun Rise and Sun Set Выводим на экран -------------------------------

void Sun( void ) { 

  byte Up_m = 0;
  byte Up_h = 0;
  byte D_m = 0;
  byte D_h = 0;

  int t;

  // =========================================================

  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {

    DateTime cur_time = rtc.now();

    Sunrise mySunrise(gps.location.lat(),gps.location.lng(),UTC);

    mySunrise.Actual();

    t = mySunrise.Rise(cur_time.month(),cur_time.day());

    if(t >= 0) {
      Up_h = mySunrise.Hour();
      Up_m = mySunrise.Minute();
    }

    t = mySunrise.Set(cur_time.month(),cur_time.day());

    if(t >= 0) {
      D_h = mySunrise.Hour();
      D_m = mySunrise.Minute();
    }

  } // Если GPS is OK

  GLCD.SelectFont(fixednums7x15);  // Select Font

  GLCD.CursorToXY(1,27);
  if (Up_h<10) GLCD.print("0"); 
  GLCD.print(Up_h,DEC);     
  GLCD.print(":");     
  if (Up_m<10) GLCD.print("0"); 
  GLCD.print(Up_m,DEC);     

  GLCD.CursorToXY(1,45);
  if (D_h<10) GLCD.print("0"); 
  GLCD.print(D_h,DEC);     
  GLCD.print(":");     
  if (D_m<10) GLCD.print("0"); 
  GLCD.print(D_m,DEC);     

}

// --------------------------------------- Сохраняем данные о давлении ------------------------------------

void Save_Bar_Data( void ) {

  DateTime now = rtc.now();

  bmp085_data.unix_time = now.unixtime(); 

  unsigned long BAR_EEPROM_POS = ( (bmp085_data.unix_time/1800)%96 ) * sizeof(bmp085_data); // Номер ячейки памяти.

  const byte* p = (const byte*)(const void*)&bmp085_data;
  for (unsigned int i = 0; i < sizeof(bmp085_data); i++) 
    EEPROM.write(BAR_EEPROM_POS++,*p++);

}

// ------------------------------- Выводим на экран данные давления ----------------------------------------

void Show_Bar_Data( void ) {

  DateTime now = rtc.now(); 

  byte current_position = (now.unixtime()/1800)%96;  

  Average<double> bar_data(96); // Вычисление максимального и минимального значения

  double barArray[96];   

  unsigned long BAR_EEPROM_POS = 0;

  for(byte j = 0;j < 96; j++) {           

    byte* pp = (byte*)(void*)&bmp085_out;   
    for (unsigned int i = 0; i < sizeof(bmp085_out); i++)
      *pp++ = EEPROM.read(BAR_EEPROM_POS++); 

    if (DEBUG) {

      DateTime eeTime (bmp085_out.unix_time);

      if ( bmp085_out.Press > 0.0 ) {

        gps_out.print(j);
        gps_out.print(" ");              
        gps_out.print(current_position);
        gps_out.print(" ");      
        gps_out.print(bmp085_out.Press);
        gps_out.print(" ");
        gps_out.print(bmp085_out.unix_time);

        gps_out.print("  ");

        gps_out.print(eeTime.year()); 
        gps_out.print("-");
        gps_out.print(eeTime.month());
        gps_out.print("-");
        gps_out.print(eeTime.day());

        gps_out.print("  ");

        gps_out.print(eeTime.hour());
        gps_out.print(":");
        gps_out.print(eeTime.minute());
        gps_out.print(":");
        gps_out.println(eeTime.second());


      }

    }

    if ((now.unixtime() - bmp085_out.unix_time) < TWO_DAYS) {
      barArray[j] = bmp085_out.Press; 
      bar_data.push(bmp085_out.Press);     
    } 
    else {      
      barArray[j] = 0.0;
    }
  }


  int y_pres = 127;
  int x;
  
  GLCD.FillRect(46,26,81,33,WHITE);

  for(byte j=0;j<82;j++) { // Количество данный выводимых на дисплей

    if (barArray[current_position] > 0.0) {     
      x = map(barArray[current_position],bar_data.minimum(),bar_data.maximum(),59,26);
      GLCD.DrawLine(y_pres,x,y_pres,59,BLACK);       // Нарисовать данные    
    }

    if (current_position == 0) current_position = 96;

    current_position--; 

    y_pres--;

  } 

}







