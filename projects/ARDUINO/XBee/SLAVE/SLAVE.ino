#include <Wire.h>
#include <string.h> 
#include <XBeeSoft.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define ZBEE_QUANTITY 8

#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3

#define THROTTLE_PIN 6
#define PITCH_PIN 9
#define ROLL_PIN 10
#define YAW_PIN 11

#define DESTINATION (0x0000)

#define DEBUG

// XBee
XBee xbee = XBee();

unsigned long xbee_timer = 0;
char incomingByte = 0;   // for incoming serial data
String receiveData[ZBEE_QUANTITY];

uint8_t nodeid = 8;

// Drone Control Order var
Servo s_throttle, s_pitch, s_roll, s_yaw;

bool is_input_pwm;
unsigned char input_length;
unsigned char input_index;
uint16_t pwm[4], pwm_temp[4];

// sonar var
int trigPin = 4;
int echoPin = 3;

float duration = 0 ; float rawDistance = 0;
unsigned long sonar_timestamp = 0;
unsigned long sonar_timer = 0;

void setup(void)
{
  Serial.begin(115200);
  xbee.begin(9600);
  is_input_pwm = false;
  input_length = 16;
  input_index = 0;

  pwm[THROTTLE] = pwm_temp[THROTTLE] = 1000;
  pwm[ROLL] = pwm[PITCH] = pwm[YAW] = pwm_temp[ROLL] = pwm_temp[PITCH] = pwm_temp[YAW] = 1500;
  s_throttle.attach(THROTTLE_PIN);
  s_pitch.attach(PITCH_PIN);
  s_roll.attach(ROLL_PIN);
  s_yaw.attach(YAW_PIN);
  
  writeMotors();
}

void writeMotors() {
  s_throttle.writeMicroseconds(pwm[THROTTLE]);
  s_pitch.writeMicroseconds(pwm[PITCH]);
  s_roll.writeMicroseconds(pwm[ROLL]);
  s_yaw.writeMicroseconds(pwm[YAW]);
}

int _10_pow(unsigned int exponent) {
  unsigned int result = 1;
  for (int i = 0; i < exponent; i++) {
    result *= 10;
  }
  return result;
}

void loop()
{
  if ( millis() - sonar_timer > 30 ) {
    sonar();
    sonar_timer = millis();
  }

  if( millis() - xbee_timer > 1000 ) {
    uint8_t payload[] = { nodeid };
    Tx16Request tx16 = Tx16Request(DESTINATION, payload, sizeof(payload));
    xbee.send(tx16);
  }
  
  writeMotors();
}

void serialEvent() {
  while (Serial.available()) {
    // read the incoming byte:
    incomingByte = Serial.read();

    if (incomingByte == '#') {
      pwm_temp[THROTTLE] = pwm_temp[ROLL] = pwm_temp[PITCH] = pwm_temp[YAW] = 0;
      input_index = 0;
    } else if (incomingByte == '@' && input_index == input_length) {
      for (int i = 0; i < 4; i++) {
        pwm[i] = pwm_temp[i];
      }
#ifdef DEBUG
      Serial.print("{");
      Serial.print("\"throttle\":");    Serial.print(pwm[THROTTLE]);  Serial.print(",");
      Serial.print("\"pitch\":");    Serial.print(pwm[PITCH]);  Serial.print(",");
      Serial.print("\"roll\":");    Serial.print(pwm[ROLL]);  Serial.print(",");
      Serial.print("\"yaw\":");    Serial.print(pwm[YAW]);
      Serial.println("}");
#endif
    } else {
      int num = incomingByte - '0';
      pwm_temp[input_index / 4] += _10_pow(3 - (input_index % 4)) * num;

      input_index++;
    }
  }
}

void sonar() {
  digitalWrite(trigPin, HIGH);
  delay(10);
  digitalWrite(trigPin, LOW);
  // echoPin 이 HIGH를 유지한 시간을 저장 한다.
  duration = pulseIn(echoPin, HIGH);
  sonar_timestamp = micros();
  // HIGH 였을 때 시간(초음파가 보냈다가 다시 들어온 시간)을 가지고 거리를 계산 한다.
  // 340은 초당 초음파(소리)의 속도, 10000은 밀리세컨드를 세컨드로, 왕복거리이므로 2로 나눠준다.
  rawDistance = ((float)(340 * duration) / 10000) / 2;

  Serial.print("{");
  Serial.print("\"sonar_distance\":");    Serial.print(rawDistance);  Serial.print(",");
  Serial.print("\"timestamp\":");    Serial.print(sonar_timestamp);
  Serial.print("}");
}
