#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstdint>
#include <cstring>
#include <vector>
#include <thread>
#include <condition_variable>
#include <stdexcept>
#include <string>
#include <iostream>
#include <mutex>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <atomic>
#include <jsoncpp/json/json.h>

class HRStateClient
{
public:
	HRStateClient();
	~HRStateClient();

public:
	//robot data
    bool robot_enable_state;
    bool robot_moving_state;
    int robot_curFSM_state;
    int robot_error_code;
	int autoMode;
	double power_on_voltage;
    std::vector<int> robot_box_do_state;
    std::vector<int> robot_box_di_state;
    std::vector<int> robot_box_co_state;
    std::vector<int> robot_box_ci_state ;
    std::vector<int> robot_end_do_state;
    std::vector<int> robot_end_di_state;

public:
	int WriteDataMode = 1;
	uint ActualDataPacketSize = 0;
private:
	std::mutex mtx;
	char recvBuf[1024];
	std::string IP;
	int PORT;
	bool ThreadSwitch = true;
	std::thread Thread_State;
public:
	int Connect(std::string ip, int port);
private:
	std::chrono::time_point<std::chrono::high_resolution_clock> ThreadTime;
	int ReceiveData();
	void OnRecvData(std::string str_Recvdata, int nLen);
};