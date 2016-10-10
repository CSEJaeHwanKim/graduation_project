#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <inttypes.h>
#include <string>

using namespace std;


class gps_serial {
private:
	int uart0_filestream;
	string portname;

public:
	gps_serial();

public:
	void serial_init(void);
	void serial_config(void);
	void serial_println(const char *, int);
	void serial_readln(char *, int);
	void serial_close(void);
};


#endif
