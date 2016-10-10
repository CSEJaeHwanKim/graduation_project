/*
* Graduation Project [ Quad Copter ]
*/

#include "./networks/UARTClass.h"	
#include "./networks/ServerSocket.h"
#include "./networks/ClientSocket.h"
#include "./networks/SocketException.h"
#include "./controllers/droneController.h"
#include "./controllers/altitudeController.h"
#include "./controllers/positionController.h"

#include <iostream>
#include <wiringPi.h>
#include <string>
#include <thread>

bool is_takeoff = false;

// UART DATA To Raspberry
// Raspberry To Web
void SendToWeb() {
	UARTClass * uart = UARTClass::getInstance();
	uart->setup();
	uart->startCommunicate();

	while (true) {
		try {
			ClientSocket client("218.150.181.154", 10000);
			while (true) {
				try {
					client << uart->getMsg(); // 웹으로 메세지 보내기
											  //cout << uart->getMsg() << endl;
					uart->clear();
				}
				catch (SocketException& e) {
					cout << "Exception was caught: " << e.description() << "\n";
				}
				delay(1000);	// 10sec 
			}
		}
		catch (SocketException& e) {
			cout << "ClientSocket Constructor Exception" << e.description() << endl;
		}
		delay(100);
	}
}

// Receive ORDER From Web
// T,Y,P,R DATA To Arduino
void RecvFromWeb() {
	string str = "";
	// UltrasonicClass * sonar = UltrasonicClass::getInstance();
	DroneController *drone = DroneController::getInstance();
	UARTClass * uart = UARTClass::getInstance();
	AltitudeController *altiController = AltitudeController::getInstance();
	PositionController *posController = PositionController::getInstance();
	while (true) {
		try {
			ServerSocket server(8800);	// Create server socket
			while (true) {
				ServerSocket new_sock;
				server.accept(new_sock);
				try {
					new_sock >> str;
					// json 방식으로 받기
					cout << str << endl;

					Document order;
					if (!order.Parse(str.data()).HasParseError()) {
						/*
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
						// cout<<"send serial : "<<pwm<<endl;
						uart->SendSerial(pwm);
						*/

						Value& web_command = order["command"];

						switch (web_command.GetInt()) {
						case 1:
							cout << "!------------ start ------------!" << endl;
							drone->order(commandType::start);
							break;

						case 2:
							cout << "!------------ stop ------------!" << endl;
							drone->order(commandType::stop);
							is_takeoff = false;
							break;

						case 3:
							cout << "!------------ landing ------------!" << endl;
							altiController->set_altitude(-0.5);
							drone->order(commandType::landing);
							//posController->stop();
							break;

						case 4:
							cout << "!------------ gps 좌표 이동 ------------!" << endl;
							altiController->set_altitude(0.4);
							
							is_takeoff = true;
														// 3. 고도 설정
														// 4. 방향 맞추기 및 gps 좌표로 이동
							const Value& gps_command_latitude = order["latitude"];
							const Value& gps_command_longitude = order["longitude"];
							double *lats = new double[gps_command_latitude.Size()];
							double *longs = new double[gps_command_latitude.Size()];
							

							for (SizeType i = 0; i < gps_command_latitude.Size(); i++) { //size_t대신 SizeType사용
								cout << "[" << i << "]" << " latitude = " << gps_command_latitude[i].GetDouble() << ", ";
								cout << " longitude = " << gps_command_longitude[i].GetDouble() << endl;
								lats[i] = gps_command_latitude[i].GetDouble();
								longs[i] = gps_command_longitude[i].GetDouble();
							}
							posController->input_coordinate(lats, longs, gps_command_latitude.Size());
							//posController->start();
							break;
						}

						str = "";
					}
				}
				catch (SocketException &e) {
					cout << "Server Socket error : " << e.description() << endl;
				}
			}
		}
		catch (SocketException& e) {
			cout << "Exception was caught: " << e.description() << "\nExiting.\n";
		}
	}
}

int main() {
	UARTClass * uart = UARTClass::getInstance();
	AltitudeController *altiCon = AltitudeController::getInstance();
	UltrasonicClass *sonar = UltrasonicClass::getInstance();
	DroneController *drone = DroneController::getInstance();

	uart->setup();
	uart->startCommunicate();

	// *************** ( ^_^ )*****************
	thread serverTh([&]() { RecvFromWeb(); });
	serverTh.detach();
	//thread clientTh([&]() { SendToWeb(); });
	//clientTh.detach();
	// *****************************************
	delay(3000);

	while (1) {
		uart->SendSerial(drone->get_pwm_string());
		cout << "throttle : " << drone->get_pwm(droneControlType::throttle) << endl;
		if (is_takeoff) {
			altiCon->calculate();
		}
		delay(100);
	}

	return 0;
}