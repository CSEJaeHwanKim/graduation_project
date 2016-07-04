#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define ADDRESS 0x04

static const char *deviceName= "/dev/i2c-1";

int main( int argc, char ** argv) {
    if( argc == 1 ) {
        printf("Usage:\n");
        printf("%s 1 : led control\n", argv[0]);
        printf("%s 2 : get temperature\n", argv[0]);
        exit(1);
    }

    int file;
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

    int arg;

    for(arg=1;arg<argc;arg++) {
        int val;
        unsigned char cmd[16];

        if( 0 == sscanf(argv[arg], "%d", &val)) {
            fprintf(stderr, "Invalid parameter %d \"%s\"\n", arg, argv[arg]);
            exit(1);
        }

        printf("Sending %d\n", val);

        cmd[0] = val;
        if(write(file, cmd, 1) == 1) {
            usleep(10000);

            char buf[1024], buf2[1024];
            read( file, buf, 1024);

            int i;
            for(i=0;i<1024;i++) {
                buf2[i] = buf[i];
                if( buf[i] == '\n' ) {
                    buf2[i] = '\0';
                    break;
                }
            }
            printf("[%s]\n", buf2);
       }
       usleep(10000);
    }
    close(file);
    return 0;
}
