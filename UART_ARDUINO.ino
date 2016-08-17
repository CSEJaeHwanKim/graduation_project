#include <XBee.h>

#include <Wire.h>
#include <string.h>
 
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_GPS.h>

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
 
// GPS
SoftwareSerial Serial1(3, 2);   // RX, TX
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
 
#define GPSECHO  true
 
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


// XBee
XBee xbee = XBee();
int digitalPin = 10;
int rssiDur;
int led = 12;
int rnodeid;
int rtemp;
int rhumi;
int node[10] = {0,};
 
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();
 
void setup(void)
{
  // Xbee ---------------------------------
  xbee.begin(9600);
  

  Serial.begin(9600);
  /* Initialise the sensor */
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  /* Display some basic information on this sensor */
  //displaySensorDetails();
 
  // GPS ----------------------------------
  mySerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
 
#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif
 
  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

}
  

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  //if (GPSECHO)
  // if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}
 
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__

void loop(void)
{
  SendData(); // airpress, GPS data Transfer
 
  delay(1000);
}

void AirpressSensor() {
  /* Get a new sensor event */
  sensors_event_t event;
  bmp.getEvent(&event);
  
  /* Display the results (barometric pressure is measure in hPa)*/
  if (event.pressure) {
    //if (false) {
    /* Display atmospheric pressue in hPa */
    //Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.print("#");
    //Serial.println(event.pressure);
    //Serial.println(" hPa");
 
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    //Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("#");
    //Serial.println(temperature);
    //Serial.println(" C");
 
    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    //Serial.print("Altitude:    ");
    Serial.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure));
    Serial.print("a");
    //Serial.println(bmp.pressureToAltitude(seaLevelPressure,event.pressure));
    //Serial.println(" m");
    //Serial.println("");
  }
  else {
    Serial.println("Sensor error");
  }
}

void XBeeSensor() {
  xbee.readPacket();
  if (xbee.getResponse().isAvailable())
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE)
    {
      xbee.getResponse().getRx16Response(rx16);
      for (int i = 0; i < rx16.getDataLength(); i++)
      {
        node[i] = rx16.getData(i);
      }
      // Serial.print("nodeid= ");
      Serial.print("+");
      Serial.print(node[0]);
      Serial.print("#");
      //Serial.print("TEMPER = ");
      Serial.print(node[1]);
      Serial.print("#");
      //Serial.print("HUMI = ");
      Serial.print(node[2]);
      Serial.print("#");
      //Serial.print(" ");
      //Serial.print("RSSI VALUE -> ");
      Serial.print(rx16.getRssi());
      Serial.print("x");
    }
  }
}

void GpsSensor() {
  // GPS ------------------------------------------------
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
 
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  } 
   
  // if millis() or timer wraps around, we'll just reset it
  //if (timer > millis())  timer = millis();
 
  // approximately every 2 seconds or so, print out the current stats
  //if (millis() - timer > 3000) {
    //timer = millis(); // reset the timer
    //Serial.print("\nTime: ");
    //Serial.print(GPS.hour, DEC); Serial.print(':');
    //Serial.print(GPS.minute, DEC); Serial.print(':');
    //Serial.print(GPS.seconds, DEC); Serial.print('.');
    //Serial.println(GPS.milliseconds);
    //Serial.print("Date: ");
    //Serial.print(GPS.day, DEC); Serial.print('/');
    //Serial.print(GPS.month, DEC); Serial.print("/20");
    //Serial.println(GPS.year, DEC);
    //Serial.print("Fix: "); Serial.print((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
 
    if (GPS.fix) {
      Serial.print("+");
      Serial.print(GPS.latitude, 4);
      Serial.print(GPS.lat);
      Serial.print("#");
 
      Serial.print(GPS.longitude, 4);
      Serial.print(GPS.lon);
      Serial.print("#");
 
      Serial.print(GPS.speed);
      Serial.print("#");
      Serial.print(GPS.angle);
      Serial.print("g");
    }
  //}
}
 
void SendData() { 
  //delay(200);
  //AirpressSensor();
  
  delay(200);
  XBeeSensor();
  
  //delay(100);
  //GpsSensor();
}
