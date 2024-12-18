#include"elfin_tcp/elfin_tcp_data.h"

HRDataClient::HRDataClient()
{
	return;
}

HRDataClient::~HRDataClient()
{
	ThreadSwitch = false;
	if (Thread_Data.joinable())
	{
		Thread_Data.join();
	}
}
int HRDataClient::Connect(std::string ip, int port)
{
	IP = ip.c_str();
	PORT = port;
	std::cout<<"Elfin TCP Data Connect by IP: "<<ip<<","<<"port: "<<port<<std::endl;
	client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket == -1) {
        std::cerr << "Failed to create socket" << std::endl;
        return -1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, IP.c_str(), &(server_addr.sin_addr));

    if (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        std::cerr << "Failed to connect  server by TCP Data" << std::endl;
        close(client_socket);
        return -1;
    }
	Thread_Data = std::thread(&HRDataClient::ReceiveData, this);
	return 0;
}
double HRDataClient::GetDouble(unsigned char * pData)
{
	double Data;
	std::memcpy(&Data, pData, sizeof(double));
	return Data;
}

uint32_t HRDataClient::Getuint32_t(unsigned char * pData)
{
	uint32_t Data;
	std::memcpy(&Data, pData, sizeof(int32_t));
	return Data;
}

int32_t HRDataClient::GetInt32_t(unsigned char * pData)
{
	int32_t Data;
	std::memcpy(&Data, pData, sizeof(int32_t));
	return Data;
}

int64_t HRDataClient::GetLongInt(unsigned char * pData)
{
	int64_t Data;
	std::memcpy(&Data, pData, sizeof(int64_t));
	return Data;
}
void HRDataClient::OnRecvData(char* pData, int nLen)
{
	std::unique_lock<std::mutex> lock(mtx);
	int offset = 0;
	int Noffset = 0;
	int Endoffset = 0;
	// DataHead = Getuint32_t((unsigned char *)(pData + offset)); offset += sizeof(DataHead);
	// 0x5242544C
	PacketSise = Getuint32_t((unsigned char *)(pData + offset)); offset += sizeof(PacketSise);
	DataLength = Getuint32_t((unsigned char *)(pData + offset)); offset += sizeof(DataLength);
    DataLength2 = Getuint32_t((unsigned char *)(pData + offset)); offset += sizeof(DataLength);

	for (int n = 0; n < 6; n++)
	{
		Noffset = offset;
		CmdPosAcs[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(CmdPosAcs);
		CmdVelAcs[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(CmdVelAcs);
		CmdAccAcs[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(CmdAccAcs);
		CmdMotorCur[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(CmdMotorCur);
		CmdTorque[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(CmdTorque);
		ActPosAcs[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(ActPosAcs);
		ActVelAcs[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(ActVelAcs);
		ActAccAcs[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(ActAccAcs);
		ActMotorCur[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(ActMotorCur);
		ActTorque[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(ActTorque);
		CmdPosPcs[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(CmdPosPcs);
		CmdVelPcs[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(CmdVelPcs);
		ActPosPcs[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(ActPosPcs);
		ActVelPcs[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(ActVelPcs);
		UserCoord[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(UserCoord);
		ToolCoord[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Endoffset=Noffset+ sizeof(UserCoord);
	}

	offset = Endoffset;
	ActEndVelPcs = GetDouble((unsigned char *)(pData + offset)); offset += sizeof(ActEndVelPcs);
	CmdEndVelPcs = GetDouble((unsigned char *)(pData + offset)); offset += sizeof(CmdEndVelPcs);
	for (int n = 0; n < 6; n++)
	{
		Noffset = offset;
		SrcForceData[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Noffset += sizeof(SrcForceData);
		FrameForceData[n] = GetDouble((unsigned char *)(pData + Noffset + n * 8)); Endoffset = Noffset + sizeof(UserCoord);
	}
	offset = Endoffset;
	CmdOverride = GetDouble((unsigned char *)(pData + offset)); offset += sizeof(CmdOverride);
	Momentum = GetDouble((unsigned char *)(pData + offset)); offset += sizeof(Momentum);
	PhysicsPower = GetDouble((unsigned char *)(pData + offset)); offset += sizeof(PhysicsPower);
	ElectricPower = GetDouble((unsigned char *)(pData + offset)); offset += sizeof(ElectricPower);
	SystemTimeStart = GetLongInt((unsigned char *)(pData + offset)); offset += sizeof(SystemTimeStart);
}

void HRDataClient::ReceiveData() {
    while (ThreadSwitch) {
        ThreadCycle = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - ThreadTime);
        ThreadTime = std::chrono::high_resolution_clock::now();
		char DataHeadData[4];
        char recvBuf[932];
		ssize_t HeadDataSize = recv(client_socket, DataHeadData, 4, 0);
		if ((HeadDataSize <= 0) || (HeadDataSize != 4)) {
            if (HeadDataSize != 4) {
                ErrorData++;
                continue;
            }
            std::cerr << "Socket error or connection closed" << std::endl;
            break;
        }
		int head_data = Getuint32_t((unsigned char *)(DataHeadData));
		if(head_data == 0x5242544C)
		{
			ssize_t ActualDataPacketSize = recv(client_socket, recvBuf, 932, 0);
			if ((ActualDataPacketSize <= 0) || (ActualDataPacketSize != 932)) {
				if (ActualDataPacketSize != 932) {
					ErrorData++;
					continue;
				}
				std::cerr << "Socket error or connection closed" << std::endl;
				break;
			}

			DataCount++;
			OnRecvData(recvBuf, ActualDataPacketSize);
		}
    }

    close(client_socket);
}