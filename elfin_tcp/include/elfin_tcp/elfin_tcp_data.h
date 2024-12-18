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

class HRDataClient
{
public:
	HRDataClient();
	~HRDataClient();

public:
	//robot data
	int DataHead;
	int PacketSise;
	int DataLength;
    int DataLength2;
	double CmdPosAcs[6];
	double CmdVelAcs[6];
	double CmdAccAcs[6];
	double CmdMotorCur[6];
	double CmdTorque[6];
	double ActPosAcs[6];
	double ActVelAcs[6];
	double ActAccAcs[6];
	double ActMotorCur[6];
	double ActTorque[6];
	double CmdPosPcs[6];
	double CmdVelPcs[6];
	double ActPosPcs[6];
	double ActVelPcs[6];
	double UserCoord[6];
	double ToolCoord[6];
	double CmdEndVelPcs;
	double ActEndVelPcs;
	double SrcForceData[6];
	double FrameForceData[6];
	double CmdOverride;
	double Momentum;
	double PhysicsPower;
	double ElectricPower;
	int64_t SystemTimeStart;

public:
	int WriteDataMode = 1;
	uint DataCount = 0;
	uint ErrorData = 0;
	uint ActualDataPacketSize = 0;
	std::chrono::microseconds ThreadCycle;
private:
	std::mutex mtx;
	char recvBuf[936];
	std::string IP;
	int PORT;
	int client_socket;
	bool ThreadSwitch = true;
	std::thread Thread_Data;
public:
	int Connect(std::string ip, int port);
private:
	std::chrono::time_point<std::chrono::high_resolution_clock> ThreadTime;
	void ReceiveData();
	double GetDouble(unsigned char * pData);
	uint32_t Getuint32_t(unsigned char * pData);
    int32_t GetInt32_t(unsigned char * pData);
	int64_t GetLongInt(unsigned char * pData);
	void OnRecvData(char* pData, int nLen);
};