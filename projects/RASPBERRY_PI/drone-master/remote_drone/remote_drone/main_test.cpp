/* 
 * Graduation Project [ Quad Copter ]
 */

#include "UARTClass.h"	
#include "ServerSocket.h"
#include "ClientSocket.h"
#include "SocketException.h"
#include "toNlanding.h"
#include "droneController.h"
#include "ultrasonic.h"

#include "airpress/AirpressClass.h"

#include <iostream>
#include <wiringPi.h>
#include <string>
#include <thread>

// UART DATA To Raspberry
// Raspberry To Web
void SendToWeb() {
	UARTClass * uart = UARTClass::getInstance();
	uart->setup();
	uart->startCommunicate();
	
	while(true) {
		try {
			ClientSocket client("218.150.181.154", 10000 );
			while(true) {
				try {
					client << uart->getMsg(); // 웹으로 메세지 보내기
					//cout << uart->getMsg() << endl;
				}
				catch ( SocketException& e ) {
				  cout << "Exception was caught: " << e.description() << "\n";
				}
				delay(1000);	// 10sec 
			}
		}
		catch( SocketException& e ) {
			cout << "ClientSocket Constructor Exception" << e.description() << endl;
		}
		delay(100);
	}
}

// Receive ORDER From Web
// T,Y,P,R DATA To Arduino
void RecvFromWeb() {
	string str = "";
	//UltrasonicClass * sonar = UltrasonicClass::getInstance();
	droneController *drone = droneController::getInstance();
	toNlanding *landing = toNlanding::getInstance();
	
	UARTClass * uart = UARTClass::getInstance();
	while(true){
		try {
			ServerSocket server( 8800 );	// Create server socket
			while ( true ) {
				ServerSocket new_sock;
				server.accept( new_sock );
				try {
					new_sock >> str;
					// json 방식으로 받기
					//cout << str << endl;
					
					Document order;
					if(!order.Parse(str.data()).HasParseError()){
					
					
					string pwm="#";
					Value& web_command=order["throttle"];
					pwm+=web_command.GetString();
					web_command=order["pitch"];
					pwm+=web_command.GetString();
					web_command=order["roll"];
					pwm+=web_command.GetString();
					web_command=order["yaw"];
					pwm+=web_command.GetString();
					pwm+="@";
					cout<<"send serial : "<<pwm<<endl;
					uart->SendSerial(pwm);
					}
					
					/*
					Value& web_command=order["command"];
					
					switch(web_command.GetString()[0]-'0') {
						case 1:
						cout<<"!------------ stop ------------!"<<endl;
						drone->stop_drone();
						landing->stop();
						break;
						
						case 2:
						cout<<"!------------ start ------------!"<<endl;
						drone->start();
						break;
						
						case 3:
						cout<<"!------------ 70cm ------------!"<<endl;
						landing->set_altitude(60);	// 설정 높이 세팅
						landing->start();	// 이륙
						break;
						
						case 4:
						cout<<"!------------ 30cm ------------!"<<endl;
						landing->set_altitude(50);	// 설정 높이 세팅
						landing->start();	// 이륙
						break;
   
						case 5:
						cout<<"!------------ 2cm ------------!"<<endl;
						landing->set_altitude(2);	// 설정 높이 세팅
						landing->start();	// 이륙
						break;
						 
					}
					*/
					str = "";
				}
				catch ( SocketException &e ) {
					cout << "Server Socket error : " << e.description() << endl;
				}
			}
		} 
		catch ( SocketException& e ) {
			cout << "Exception was caught: " << e.description() << "\nExiting.\n";
		}
	}
}
	
int main() {	
	UARTClass * uart = UARTClass::getInstance();
	UltrasonicClass *sonar = UltrasonicClass::getInstance();
	droneController *drone = droneController::getInstance();
	uart->setup();
	uart->startCommunicate();
	// *************** ( ^_^ )*****************
	//thread serverTh([&]() { RecvFromWeb(); });
	//serverTh.detach();
	thread clientTh([&]() { SendToWeb(); });
	clientTh.detach();
	// *****************************************
	delay(3000);
	
	while(1){
		//uart->SendSerial(drone->get_pwm_string());
		//cout << "throttle : " << drone->get_pwm(droneControlType::throttle) <<" , distance : " << sonar->getDistance()<<endl;
		delay(10);
	}
	
	return 0;
}