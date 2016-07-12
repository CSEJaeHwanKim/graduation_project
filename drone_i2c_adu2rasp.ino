// include the pinchangeint library - see the links in the related topics section above for details
#include <PinChangeInt.h>
#include <SPI.h>
#include <Servo.h>

//LED pin for checking
#define LED_PIN 13

//For ease of programming
#define THR 0
#define YAW 1
#define PITCH 2
#define ROLL 3

// Assign your channel in pins
#define THROTTLE_IN_PIN 8
#define YAW_IN_PIN 11
#define PITCH_IN_PIN 10
#define ROLL_IN_PIN 9

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unYawInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulYawStart;
uint32_t ulPitchStart;
uint32_t ulRollStart;


//servo variables
//Number of servos
#define SERVO_NUM 4
#define RC_MIN 1000
// Assign your channel out pins
#define FL_MOTOR_OUT_PIN 4
#define FR_MOTOR_OUT_PIN 5
#define BL_MOTOR_OUT_PIN 6
#define BR_MOTOR_OUT_PIN 7

//define Servo variables
Servo MOTOR[SERVO_NUM];

//define SPI data
volatile union int_byt{
  uint8_t u8[2];
  uint16_t u16;
} rc_data[4], esc_data[4];

volatile byte rx_buf[SERVO_NUM*2+1];
uint8_t pos=0;
uint8_t checksum=0;

bool start=false;
volatile bool update_servo=false;

//setup function
void setup()
{
  Serial.begin(9600); //for debugging...

  pinMode(LED_PIN, OUTPUT);

  /*
    PWM measurement settings
  */

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(YAW_IN_PIN, calcYaw,CHANGE);
  PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch,CHANGE);
  PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll,CHANGE);

  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers
  MOTOR[0].attach(FL_MOTOR_OUT_PIN);
  MOTOR[1].attach(FR_MOTOR_OUT_PIN);
  MOTOR[2].attach(BL_MOTOR_OUT_PIN);
  MOTOR[3].attach(BR_MOTOR_OUT_PIN);

  //Set servo values to min
  for (int i=0;i<SERVO_NUM;i++)
    {
      MOTOR[i].writeMicroseconds(RC_MIN);
    }

  /* receive data
    SPI settings
  */
  // Declare MISO as output : have to send on
  //master in, *slave out*
  pinMode(MISO, OUTPUT);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // now turn on interrupts
  SPI.attachInterrupt();
  /* send data
     SPI setting
  */
  SPI.begin(); // SPI initailize
  digitalWrite(SS, HIGH); // 슬레이브가 선택되지 않은 상태로 유지
  
  // 안정적인 전송을 위해 분주비를 높여 전송 속도를 낮춤
  SPI.setClockDivider(SPI_CLOCK_DIV16);
}

void loop()
{
  //Constantly update rc_data
  rc_data[THR].u16 = unThrottleInShared;
  rc_data[YAW].u16 = unYawInShared;
  rc_data[PITCH].u16 = unPitchInShared;
  rc_data[ROLL].u16 = unRollInShared;

  if (unThrottleInShared < 1000 &
      unYawInShared      < 1200 &
      unPitchInShared    < 1200 &
      unRollInShared     > 1200 ) {

    uint32_t t_old = millis();
    while (millis()-t_old < 500 ){
      //wait to be sure that sticks are still in position
    }

    // if so change current status
    if (unThrottleInShared < 1000 &
	unYawInShared      < 1200 &
	unPitchInShared    < 1200 &
	unRollInShared     > 1200 ) {

      start = !start;
      //change LED status
      digitalWrite(13, start);
    }
  }

  //Update servo (ESC) if necessary and started
  if (update_servo & start){
    uint8_t ipos=0;
    byte checksum2=0;
    for (int i=0;i<SERVO_NUM;i++)
      {
	//process buffer values
	for (int ibyte = 0;ibyte<2;ibyte++){
	  esc_data[i].u8[ibyte] = rx_buf[ipos];
	  checksum2+=rx_buf[ipos];
	  ipos++;
	}
      }

    if (rx_buf[ipos] == checksum2) {
      //write ESC output
      for (int i=0;i<SERVO_NUM;i++)
	MOTOR[i].writeMicroseconds(esc_data[i].u16);
    }

    update_servo = false;

  } else if (!start) {
    for (int i=0;i<SERVO_NUM;i++)
      {
	MOTOR[i].writeMicroseconds(RC_MIN);
      }
  }

}

/*-----------------------
  SPI interrupt routine
------------------------*/
// Slave source
// when receive the data, interupt service rountine 
ISR (SPI_STC_vect)
{

  //grab a new command and process it
  byte cmd = SPDR;

  if (cmd == 'S') {
    //STOP do nothing : end of sending RC data
    pos=0;
    return;
  }

  if (cmd == 'P') {
    //process it !
    update_servo = true;
    pos=0;
    return;
  }

  if (cmd == 'C') {
    //push Cheksum into the register
    SPDR = checksum;
    checksum = 0;
    pos=0;
    return;
  }


  //push data into a byte buffer
  rx_buf[pos++] = cmd;

  // 10-11 -> send 2bytes channel 1 value
  // ...
  // 40-41 -> send 2bytes channel 4 value
  // ...
  // 60-61 -> send 2bytes channel 6 value
  switch (cmd){
    // throttle 1
  case 10:
    SPDR = rc_data[THR].u8[0];
    checksum += rc_data[THR].u8[0];
    break;

    // throttle 2
  case 11:
    SPDR = rc_data[THR].u8[1];
    checksum += rc_data[THR].u8[1];
    break;

    // yaw 1
  case 20:
    SPDR = rc_data[YAW].u8[0];
    checksum += rc_data[YAW].u8[0];
    break;

    // yaw 2
  case 21:
    SPDR = rc_data[YAW].u8[1];
    checksum += rc_data[YAW].u8[1];
    break;

    // pitch 1
  case 30:
    SPDR = rc_data[PITCH].u8[0];
    checksum += rc_data[PITCH].u8[0];
    break;

    // pitch 2
  case 31:
    SPDR = rc_data[PITCH].u8[1];
    checksum += rc_data[PITCH].u8[1];
    break;

    // roll 1
  case 40:
    SPDR = rc_data[ROLL].u8[0];
    checksum += rc_data[ROLL].u8[0];
    break;

    // roll 2
  case 41:
    SPDR = rc_data[ROLL].u8[1];
    checksum += rc_data[ROLL].u8[1];
    break;
 }
 
 /* ------------------------------
 * Master receive data part
 * SPI 
 ----------------------------------*/
  if(Serial.available()){
    char data = Serial.read(); // 데이터 입력 확인
    if(data == 'K'){
      digitalWrite(SS, LOW); // 슬레이브를 선택한다.
      // 1바이트 데이터 수신을 위해 의미 없는 1바이트 데이터를 전송한다.
      char received = SPI.transfer(0);
      digitalWrite(SS, HIGH); // 슬레이브 선택을 해제한다.
      Serial.println(received);
    }
  }
}

/*----------------------------------
  simple interrupt service routine
  for PWM measurements
*/
void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
    {
      ulThrottleStart = micros();
    }
  else
    {
      // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
      // this gives use the time between the rising and falling edges i.e. the pulse duration.
      unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);

    }
}

void calcYaw()
{
  if(digitalRead(YAW_IN_PIN) == HIGH)
    {
      ulYawStart = micros();
    }
  else
    {
      unYawInShared = (uint16_t)(micros() - ulYawStart);
    }
}

void calcPitch()
{
  if(digitalRead(PITCH_IN_PIN) == HIGH)
    {
      ulPitchStart = micros();
    }
  else
    {
      unPitchInShared = (uint16_t)(micros() - ulPitchStart);
    }
}

void calcRoll()
{
  if(digitalRead(ROLL_IN_PIN) == HIGH)
    {
      ulRollStart = micros();
    }
  else
    {
      unRollInShared = (uint16_t)(micros() - ulRollStart);
    }
}
