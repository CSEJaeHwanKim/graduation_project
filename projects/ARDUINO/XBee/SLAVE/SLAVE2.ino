#include <XBee.h>
#include<DHT11.h>
#define MG_PIN (0) // analog pin 0
#define DC_GAIN (8.5)  
#define READ_SAMPLE_INTERVAL (50)   
#define READ_SAMPLE_TIMES (5)
#define ZERO_POINT_VOLTAGE (0.135) // sensor each different 
#define REACTION_VOLTGAE (0.010)
#define DESTINATION (0x0000)
#define SensorINPUT 3 //pinnumber

XBee xbee = XBee();
float CO2Curve[3] = {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};
int temppin = 2;
DHT11 dht11(temppin);
int nodeid = 5;
int percentage1 = 0;
int percentage2 = 0;
int percentage3 = 0;
int percentage4 = 0;
int analogPin = 1;    // 워터센서 analog port 1 연결 선언
unsigned char waterSensor = 0;          // 전류변화값 변수선언
unsigned char vibration = 0;

unsigned long vibration_timer = 0;

int greenLedPin = 4; ///greenledpin
int yellowLedPin = 5; //yellowledpin
int redLedPin = 6; //redledpin

void setup()
{
  xbee.begin(9600); 
  Serial.begin(9600);  
  pinMode(SensorINPUT, INPUT); 
  attachInterrupt(1, vibrationEvent, FALLING); 
  pinMode(greenLedPin, OUTPUT); 
  pinMode(yellowLedPin, OUTPUT); 
  pinMode(redLedPin, OUTPUT); 
}
 
void loop()
{
    int percentage;
    int co2ppm;
    float volts;
    float temp,humi;
    int temp_1,humi_1;
    
    volts = MGRead(MG_PIN);
    percentage = MGGetPercentage(volts,CO2Curve);
    if(percentage >10000)
    {
      percentage = 10000;
      co2ppm = percentage / 40;
    }
    Serial.print("CO2:   ");
    Serial.println(co2ppm);
    if(dht11.read(humi, temp)== 0)  
    {
      temp_1 = (int) temp;
      humi_1 = (int) humi;
    }
    if(millis() - vibration_timer > 2000)
    {
      vibration_timer = millis();
      vibration = 0;
    }
    if( temp_1 < 30 || vibration < 5 || waterSensor < 100 )
    {
      digitalWrite(redLedPin, LOW);
      digitalWrite(yellowLedPin, LOW);
      digitalWrite(greenLedPin, HIGH);
    }
    if( (temp_1 >= 30 && temp_1 < 40) || (vibration >= 5 && vibration < 150) || (waterSensor >= 100 && waterSensor < 200))
    {
      digitalWrite(greenLedPin, LOW);
      digitalWrite(redLedPin, LOW);
      digitalWrite(yellowLedPin, HIGH);
    }
    if(temp_1 >= 40 || vibration >= 150 || waterSensor >= 200 )
    {
      digitalWrite(greenLedPin, LOW);
      digitalWrite(yellowLedPin, LOW);
      digitalWrite(redLedPin, HIGH);
    }
    Serial.print("temp:");
    Serial.println(temp_1);
    Serial.print("humi:");
    Serial.println(humi_1);
    Serial.print("vibration:");
    Serial.println(vibration);
    Serial.print("water:");
    Serial.println(waterSensor);
    uint8_t payload[]={ nodeid, co2ppm, temp_1, humi_1, vibration, waterSensor};
    Tx16Request tx16 = Tx16Request(DESTINATION, payload, sizeof(payload)); 
    xbee.send(tx16);
    delay(2000); 
}

float MGRead(int mg_pin)
{
  int i;
  float v=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++) 
  {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    v = (v/READ_SAMPLE_TIMES) *5/1024 ;
    return v;  
}
int  MGGetPercentage(float volts, float *pcurve)
{
   if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) 
   {
      return -1;
   } 
   else 
   { 
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}
void vibrationEvent()
{
    vibration++;
}
