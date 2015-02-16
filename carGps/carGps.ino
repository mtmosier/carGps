#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include "U8glib.h"
#include <Adafruit_LSM303.h>
#include <Time.h>
#include <Timezone.h>
#include <EEPROM.h>


#define ANALOG_PIN                          A1  // Pin of the potentiometer which controls brightness
#define INT_NUMBER                           0  // Interrupt to use for the button (0 = digital pin 2)
#define NEO_PIXEL_PIN                        3  // Data output pin on the arduino
#define PIXEL_COUNT                         24  // NeoPixel Count
#define DEST_COUNT                           6  // Number of saved destinations
#define NAVMODE_ADDR                         0  // EEPROM address for the navMode value
#define DEST_ADDR                            1  // EEPROM address for the curDest value
#define SS_RX_PIN                            8  // Software Serial RX Pin
#define SS_TX_PIN                            9  // Software Serial TX Pin
#define COMPASS_OFFSET                       0  // The degress to adjust the compass heading (physical compass, not gps)
#define GPS_HEADING_TRAVEL_DISTANCE         15  // The distance between gps readings when determining heading (in meters)
#define PI                             3.14159




typedef struct
{
  float latitude;
  float longitude;
  unsigned long timeRecorded;
} Coordinates;

typedef struct
{
  const char *name;
  float latitude;
  float longitude;
  uint32_t color;
  Timezone* timezone;
} Destination;




//  Time zone setup
//US Central Time Zone (Chicago, Houston)
TimeChangeRule usCDT = {"CDT", Second, dowSunday, Mar, 2, -300};
TimeChangeRule usCST = {"CST", First, dowSunday, Nov, 2, -360};
Timezone usCT(usCDT, usCST);

/*
TimeChangeRule usMDT = {"MDT", Second, dowSunday, Mar, 2, -360};
TimeChangeRule usMST = {"MST", First, dowSunday, Nov, 2, -420};
Timezone usMT(usMDT, usMST);
*/

Destination destList[DEST_COUNT] = {
  { "Home", 29.662952, -95.323942, 0xFF0000, &usCT },  //  Red
  { "Midtown", 29.739123, -95.375583, 0xFFFF00, &usCT },  //  Yellow
  { "Heights", 29.801489, -95.409256, 0xFF00FF, &usCT },  //  Magenta
  //{ "Memorial", 29.763149, -95.425636, 0x00FF00, &usCT },  //  Green
  { "Menil", 29.737185, -95.397685, 0xFFFFFF, &usCT },  //  White
  { "Abe", 29.688889, -95.685213, 0x00FFFF, &usCT },  //  Cyan
  { "The Village", 29.6402369, -95.3281248, 0x00FF00, &usCT }  //  Green
};

const char* const bearingArray[] = {
  "  North", "North-East", "   East", "South-East", "  South", "South-West", "   West", "North-West"
};




Timezone *compassTimezone = &usCT;
boolean fixFound = false;
Coordinates cmp1Loc = {},
            cmp2Loc = {},
            curLoc = {};
volatile boolean navMode = false;
volatile uint8_t curDest = 0;
uint8_t brightness = 255;
float compassHeading = 0;
boolean compassHeadingForce = false;
unsigned long timer = millis();
uint8_t pixelIdx = 0;
boolean timeInitialized = false;
uint8_t lastLedIdx = 0;
const char* curBearing;
const uint32_t compassColor = 0x0000FF;  //  Blue
boolean usingGpsCompass = false;



// NeoPixel Setup
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(PIXEL_COUNT, NEO_PIXEL_PIN);


//  GPS setup
SoftwareSerial ss(SS_RX_PIN, SS_TX_PIN);
Adafruit_GPS GPS(&ss);


// Compass setup
Adafruit_LSM303 mag;


//  Display setup
U8GLIB_SSD1306_128X64 display(U8G_I2C_OPT_NONE);	// I2C / TWI 



void
setup()
{
  navMode = (EEPROM.read(NAVMODE_ADDR) == 1);
  curDest = EEPROM.read(DEST_ADDR);
  if (curDest >= DEST_COUNT) {
    curDest = 0;
  }

  attachInterrupt(INT_NUMBER, buttonPress, RISING);

  //display.setFont(u8g_font_helvR12);
  display.setFont(u8g_font_10x20);


  //  NeoPixel initialization
  pixels.begin();
  clearPixels();


  //  Compass initialization
  mag.begin();


  //  GPS initialization
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

}


void
loop()
{
  int analogValue = 255 - (analogRead(ANALOG_PIN) / 4);
  if (analogValue != brightness) {
    brightness = analogValue;
    pixels.setBrightness(brightness);
    pixels.show();
  }

  boolean gpsDataParsed = true;
  if (GPS.newNMEAreceived())
    gpsDataParsed = GPS.parse(GPS.lastNMEA());

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every half second or so update
  if (millis() - timer > 500) {
    timer = millis(); // reset the timer

    if (gpsDataParsed) {
      updateGpsData();
    }
    updateCompassHeading();

    if (navMode) {
      if (fixFound) {
        float bearing = calcBearing(curLoc.latitude, curLoc.longitude, destList[curDest].latitude, destList[curDest].longitude);
        if (bearing - compassHeading > 0)
          applyCompassDirection(bearing - compassHeading);
        else
          applyCompassDirection(bearing - compassHeading + 360);
      } else {
        clearPixels();
      }
    } else {
      applyCompassDirection(compassHeading);
    }

    updateDisplay();
  }
}


SIGNAL(TIMER0_COMPA_vect) {
  GPS.read();
}


void
buttonPress()
{
  if (navMode) {
    curDest++;
    if (curDest >= DEST_COUNT) {
      navMode = false;
    }
  } else {
    navMode = true;
    curDest = 0;
  }

  EEPROM.write(NAVMODE_ADDR, navMode ? 1 : 0);
  EEPROM.write(DEST_ADDR, curDest);
}


void
updateCompassHeading()
{
  float lastCompassHeading = compassHeading;
  usingGpsCompass = false;

  if (cmp1Loc.timeRecorded >= 1 && cmp2Loc.timeRecorded >= 1) {
    if (calcDist(cmp2Loc.latitude, cmp2Loc.longitude, cmp1Loc.latitude, cmp1Loc.longitude) >= GPS_HEADING_TRAVEL_DISTANCE) {
      compassHeading = calcBearing(cmp2Loc.latitude, cmp2Loc.longitude, cmp1Loc.latitude, cmp1Loc.longitude);
      usingGpsCompass = true;
    }
  }

  if (!usingGpsCompass) {
    // Get a new sensor event
    mag.read();

    compassHeading = ((atan2(mag.magData.y, mag.magData.x) * 180) / PI) - COMPASS_OFFSET;
  }

  // Normalize to 0-360
  if (compassHeading < 0) {
    compassHeading = 360 + compassHeading;
  }

  if (!compassHeadingForce && calcBearingDifference(compassHeading, lastCompassHeading) > 100) {
    compassHeading = lastCompassHeading;
    compassHeadingForce = true;
  } else {
    compassHeadingForce = false;
  }
}


void
updateGpsData()
{
  if (GPS.fix) {
    //  Sometimes gps returns 0 for one of the values, so throw those away
    if ((GPS.latitudeDegrees > 0.1 || GPS.latitudeDegrees < -0.1) && (GPS.longitudeDegrees > 0.1 || GPS.longitudeDegrees < -0.1)) {
      fixFound = true;

      curLoc.latitude = GPS.latitudeDegrees;
      curLoc.longitude = GPS.longitudeDegrees;
      curLoc.timeRecorded = millis();
      if (cmp1Loc.timeRecorded < 1 || calcDist(curLoc.latitude, curLoc.longitude, cmp1Loc.latitude, cmp1Loc.longitude) >= GPS_HEADING_TRAVEL_DISTANCE || millis() - cmp1Loc.timeRecorded > 300000) {
        memcpy(&cmp2Loc, &cmp1Loc, sizeof(cmp1Loc));
        memcpy(&cmp1Loc, &curLoc, sizeof(curLoc));
      }

      setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);
      timeInitialized = true;
      GPS.fix = false;
    }
  } else {
    if (!timeInitialized) {
      if (GPS.hour != 0 || GPS.minute != 0 || GPS.seconds != 0) {
        setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);
        timeInitialized = true;
      }
    }
  }
}


void
updateDisplay()
{
  display.firstPage();
  do {
    draw();
  } while (display.nextPage());
}


void
draw()
{
  if (timeInitialized) {
    time_t local;
    if (navMode) {
      local = destList[curDest].timezone->toLocal(now());
    } else {
      local = compassTimezone->toLocal(now());
    }

    display.setPrintPos(25, 20);

    if (hourFormat12(local) < 10)  display.print(F(" "));
    display.print(hourFormat12(local));
    display.print(F(":"));
    if (minute(local) < 10)  display.print(F("0"));
    display.print(minute(local));
    if (isAM(local))  display.print(F("am"));
    else  display.print(F("pm"));
  }

  display.setPrintPos(0, 45);
  if (!fixFound)         display.print(F("*"));
  if (navMode) {
    display.print(destList[curDest].name);
  } else {
    display.setPrintPos(15, 45);
    display.print(curBearing);
  }
}  


void
clearPixels()
{
  for (uint8_t i = 0; i < PIXEL_COUNT; i++) {
    pixels.setPixelColor(i, 0x000000);
  }
  pixels.show();
}


void
applyCompassDirection(float heading) 
{
  static const float ledWidthInDegress = 360 / PIXEL_COUNT;
  static const float halfLedWidthInDegress = ledWidthInDegress / 2;

  int ledIdx = 0;
  int bearingIdx = 0;
  uint32_t color = 0;

  float tmpHeading = heading + halfLedWidthInDegress;
  if (tmpHeading > 360) {
    tmpHeading -= 360;
  }

  ledIdx = floor(tmpHeading / ledWidthInDegress);
  if (navMode) {
    color = destList[curDest].color;
  } else {
    ledIdx = -1 * ledIdx;
    color = compassColor;
  }
  
  ledIdx = (ledIdx + PIXEL_COUNT) % PIXEL_COUNT;

  clearPixels();
  pixels.setPixelColor(ledIdx, color);
  pixels.show();
  lastLedIdx = ledIdx;

  tmpHeading = heading + 22.5;
  if (tmpHeading > 360) {
    tmpHeading -= 360;
  }

  bearingIdx = (int)(floor(tmpHeading / 45)) % 8;
  curBearing = bearingArray[bearingIdx];
}


float
calcBearing(float fromLat, float fromLong, float toLat, float toLong)
{
  float x = 69.1 * (toLat - fromLat); 
  float y = 69.1 * (toLong - fromLong) * cos(fromLat / 57.3);
  float bearCalc = degrees(atan2(y,x));

  if (bearCalc <= 1) {
    bearCalc = 360 + bearCalc; 
  }

  return bearCalc;
}


unsigned long
calcDist(float fromLat, float fromLong, float toLat, float toLong)
{
  float distCalc = 0;
  float distCalc2 = 0;
  float diflat = 0;
  float diflon = 0;

  diflat = radians(toLat - fromLat);
  fromLat = radians(fromLat);
  toLat = radians(toLat);
  diflon = radians(toLong - fromLong);

  distCalc = (sin(diflat / 2.0) * sin(diflat / 2.0));
  distCalc2 = cos(fromLat);
  distCalc2 *= cos(toLat);
  distCalc2 *= sin(diflon / 2.0);
  distCalc2 *= sin(diflon / 2.0);
  distCalc += distCalc2;

  distCalc = (2 * atan2(sqrt(distCalc), sqrt(1.0-distCalc)));

  distCalc *= 6371000.0; //Converting to meters
  return distCalc;
}


float
calcBearingDifference(float bearing1, float bearing2)
{
  float angle = abs(bearing1 - bearing2);
  if (angle > 180)  angle = 360 - angle;
  return angle;
}

