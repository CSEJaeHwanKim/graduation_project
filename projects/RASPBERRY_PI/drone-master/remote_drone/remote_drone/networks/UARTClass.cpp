#include "UARTClass.h"
#include <thread>

UARTClass::UARTClass() {
   this->is_end = false;
   this->recvStr = "";
   this->message = "";
   
   device = "/dev/ttyACM0";
   this->baud = 115200;
   this->m_time = 0;   
   
   this->fd = 0;

  instance_gps = gps::getInstance();  
  sonar = UltrasonicClass::getInstance();
}

UARTClass::~UARTClass() {}

string UARTClass::getMsg() {
	lock_guard<mutex> lock(msg_mutex);
   return this->message;
}
/*
string* UARTClass::strSplit(string strTarget, string strTok) {
    int     nCutPos;
    int     nIndex     = 0;
    string* strResult = new string[100];
 
    while ((nCutPos = strTarget.find_first_of(strTok)) != strTarget.npos)
    {
        if (nCutPos > 0)
        {
            strResult[nIndex++] = strTarget.substr( 0, nCutPos );
        }
        strTarget = strTarget.substr(nCutPos+1);
    }
  
    if(strTarget.length() > 0)
    {
        strResult[nIndex++] = strTarget.substr(0, nCutPos);
    }
 
    return strResult;
}
*/

void UARTClass::setup() {
  printf("%s \n", "Raspberry Startup!");
  fflush(stdout);
  
  instance_gps->gps_on();
   
  //get filedescriptor
  if ((fd = serialOpen (device.c_str(), baud)) < 0) {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
    exit(1); //error
  }
  
  /*
  //setup GPIO in wiringPi mode
 if (wiringPiSetup () == -1) {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    exit(1); //error
  }
  */
 
}

// send Message
void UARTClass::SendSerial( string msg ) {
   serialPuts ( fd, msg.data() );
}

// receive Message
void UARTClass::loop() {
   // read signal from Arduino
	if(serialDataAvail (fd)) {
	  char newChar = serialGetchar(fd);
	  
	  if( newChar == '{' ) {
		 is_end = true;
	  }
	  if( is_end ) {
		 recvStr += newChar;
	  }
	  //cout<<newChar;
	  if( newChar == '}' ) {
		 // Json Reader
		 // 아두이노로 부터 오는 데이터 파싱
		 
		    //cout<<recvStr.data()<<endl;
		 Document reader;
		 if (reader.Parse(recvStr.data()).HasParseError()) {
			 //cout<<"UART JSON ERROR!!!!!  = > "<<recvStr.data()<<endl;
			 recvStr = "";
		 serialFlush( fd );
		 is_end = false;
       		return;
       	}
	 
		 /*
		 Value& temp_val = reader["throttle"];
		 cout<<"throttle : "<<temp_val.GetInt()<<endl;
		 temp_val = reader["pitch"];
		 cout<<"pitch : "<<temp_val.GetInt()<<endl;
		 temp_val = reader["roll"];
		 cout<<"roll : "<<temp_val.GetInt()<<endl;
		 temp_val = reader["yaw"];
		 cout<<"yaw : "<<temp_val.GetInt()<<endl;
		 */
		 // DOM에 데이터 추가..
		if(reader.HasMember("sonar_distance")) {
			Value& dist = reader["sonar_distance"];
			Value& time = reader["timestamp"];
			sonar->input_values(dist.GetDouble(),time.GetUint());
		} else if(reader.HasMember("throttle")) {
			Value& temp_val2 = reader["throttle"];
			cout<<"throttle : "<<temp_val2.GetInt()<<endl;
			temp_val2 = reader["pitch"];
			cout<<"pitch : "<<temp_val2.GetInt()<<endl;
			temp_val2 = reader["roll"];
			cout<<"roll : "<<temp_val2.GetInt()<<endl;
			temp_val2 = reader["yaw"];
			cout<<"yaw : "<<temp_val2.GetInt()<<endl;
			 
		} else {
			 Value latitude(instance_gps->get_latitude());
			 Value longitude(instance_gps->get_longitude());
			 Value speed(instance_gps->get_speed());
			 Value course(instance_gps->get_course());
			 //Value pressure(instance_airpress->getPressure());
			 //Value temperature(instance_airpress->getTemperature());
			 //Value altitude(instance_airpress->getAltitude());

			 reader.AddMember("Latitude", latitude, reader.GetAllocator());
			 reader.AddMember("Longitude", longitude, reader.GetAllocator());
			 reader.AddMember("Speed", speed, reader.GetAllocator());
			 reader.AddMember("Course", course, reader.GetAllocator());
			// reader.AddMember("Pressure", pressure, reader.GetAllocator());
			// reader.AddMember("Temperature", temperature, reader.GetAllocator());
			// reader.AddMember("Altitude", altitude, reader.GetAllocator());
					 
			 // stringify the DOM
			 Writer<StringBuffer> writer(str_buf);
			 reader.Accept(writer);
			 
			 // message 에 담아 web으로 	 
			 {
				lock_guard<mutex> lock(msg_mutex);
				message = str_buf.GetString();
			 }
		 }
		 recvStr = "";
		 serialFlush( fd );
		 is_end = false;
	  }
	  fflush(stdout);
	}
}

void UARTClass::startCommunicate() {
   thread t([&]() { while(true) { loop(); } } );
   t.detach();
}

void UARTClass::clear()
{
	message = "";
}
