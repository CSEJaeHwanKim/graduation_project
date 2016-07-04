#include <Wire.h>  
  
//I2C 주소 지정    
#define SLAVE_ADDRESS 0x04  
int number = 0;  
int state = 0;  
int temp;  
char retMSG[1024] = "";  
  
const int ledPIN = 13;  
const int cdsPIN = 0;  
   
void setup() {  
   pinMode( ledPIN, OUTPUT);  
     
   //슬레이브로써 I2C를 초기화한다.  
   Wire.begin(SLAVE_ADDRESS);  
     
   //I2C 통신을 위한 콜백함수를 지정한다.  
   Wire.onReceive(receiveData); //데이터 수신용  
   Wire.onRequest(sendData);    //데이터 전송  
}  
  
   
void loop() {  
   delay(100);  
   temp = analogRead(cdsPIN);  
}  
   
  
void receiveData(int byteCount){  
   
 while(Wire.available()) {  
    number = Wire.read();  
     
    if (number == 1){  
         if (state == 0){  
             digitalWrite( ledPIN, HIGH ); // set the LED on  
             state = 1;  
             strcpy( retMSG, "Trun on LED\n");  
         } else{  
             digitalWrite(13, LOW); // set the LED off  
             state = 0;  
             strcpy(retMSG, "Turn off LED\n");  
           }  
     }     
    else if(number==2) {  
         sprintf( retMSG, "Light = %d\n", temp );  
    }  
    else{  
        sprintf( retMSG, "Invaild command\n" );  
    }  
      
      
   }  
}  
  
void sendData(){  
    Wire.write( retMSG, strlen(retMSG) );  
} 
