#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <wiringPi.h>
#define ADDRESS 0x04

static const char *deviceName= "/dev/i2c-1";

int main( int argc, char ** argv) {
    // socket part
    




    // ultrasonic area ****************************
    int trig = 5;
    int echo = 4;
    int start_time, end_time;
    float distance ;

    if ( wiringPiSetup() == -1) exit(1);

    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    
    // *********************************************

    int file; // fd
    if((file = open(deviceName, O_RDWR)) < 0 ) {
        fprintf(stderr, "I2C: Failed to access %s \n", deviceName);
        exit(1);
    }
    printf("I2C: Connected\n");

    printf("I2C: acquiring bus to 0x%x\n", ADDRESS);
    if(ioctl(file, I2C_SLAVE, ADDRESS) < 0 ) {
        fprintf(stderr, "I2C : Failed to acquire bus access/talk to slave 0x%x\n" );
        exit(1);
    }

    while(1) {
        digitalWrite(trig, LOW);
        delay(500);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        while( digitalRead(echo) == 0 );
        start_time = micros();
        while( digitalRead(echo) == 1 );
        end_time = micros();
        distance = (end_time - start_time) / 58;
        printf("distance %.2f cm\n", distance);

        // sending datas part ( throttle, pitch, roll, yaw )
        // put val in char type
        unsigned char m_ultrasonic[20];
        m_ultrasonic[0] = distance; // testing with ultrasonic haha
        write(file, m_ultrasonic, sizeof(m_ultrasonic));    // sending &&&
        printf("sending ultrasonic value \n");   
    }

    // need to consider filling out read part !!
   



    close(file);
    return 0;
}
