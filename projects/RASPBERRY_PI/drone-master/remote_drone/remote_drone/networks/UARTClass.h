#ifndef UART_H
#define UART_H
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <errno.h>
#include <mutex>

#include "../sensors/sonar/ultrasonic.h"
// wiring Pi
#include <wiringPi.h>
#include <wiringSerial.h>

#include "../singleton.hpp"
// json
#include "../rapidjson/document.h"
#include "../rapidjson/writer.h"
#include "../rapidjson/stringbuffer.h"
#include "../rapidjson/reader.h"

#include "../sensors/gps/gps.h"

using namespace std;
using namespace rapidjson;

class UARTClass : public singleton<UARTClass> {
   private :
      string device;
      string recvStr;
      int fd;   
      unsigned long baud;
      unsigned long m_time;
      string message;
      bool is_end;
      
      // json writer
      StringBuffer str_buf;
	  
	  gps *instance_gps;
	  
	  UltrasonicClass *sonar;
	  mutex msg_mutex;
   public :
      UARTClass();
      virtual ~UARTClass();      
      //string *strSplit( string strTarget, string strTok );
      void setup();
      void SendSerial( string msg );
      void loop(); // getMessage from Arduino
      void startCommunicate();
	  void clear();
      
      string getMsg();	  
};

#endif        /* UART_H */