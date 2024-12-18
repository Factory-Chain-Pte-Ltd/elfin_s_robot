#include "elfin_tcp/elfin_tcp_state.h"

HRStateClient::HRStateClient()
{
    robot_box_do_state.resize(8,0);
    robot_box_di_state.resize(8,0);
    robot_box_co_state.resize(8,0);
    robot_box_ci_state.resize(8,0);
    robot_end_do_state.resize(4,0);
    robot_end_di_state.resize(4,0);
    return;
}

HRStateClient::~HRStateClient()
{
    ThreadSwitch = false;
    if(Thread_State.joinable())
    {
        Thread_State.join();
    }
}

int HRStateClient::Connect(std::string ip, int port)
{
    IP = ip.c_str();
    PORT = port;
    std::cout<<"Elfin TCP State Connect by IP: "<<ip<<","<<"port: "<<port<<std::endl;
    Thread_State = std::thread(&HRStateClient::ReceiveData,this);
    return 0;
}

void HRStateClient::OnRecvData(std::string str_Recvdata, int nLen)
{
    Json::Value jsonData;
    Json::Reader reader;
    bool strJsonRes = reader.parse(str_Recvdata, jsonData);
    if(!strJsonRes)
    {
        return;
    }
    robot_enable_state = jsonData["StateAndError"]["robotEnabled"].asBool();
    robot_moving_state = jsonData["StateAndError"]["robotMoving"].asBool();
    robot_curFSM_state =  jsonData["StateAndError"]["robotState"].asInt();
    robot_error_code = jsonData["StateAndError"]["Error_Code"].asInt();
    power_on_voltage =std::stod(jsonData["HardLoad"]["Box48Out_Voltage"].asString());
    autoMode = std::stoi(jsonData["StateAndError"]["AutoMode"].asString());
    for(int i=0;i<jsonData["ElectricBoxIO"]["BoxDO"].size();i++)
    {
        robot_box_do_state[i] = jsonData["ElectricBoxIO"]["BoxDO"][i].asInt();
        robot_box_di_state [i] = jsonData["ElectricBoxIO"]["BoxDI"][i].asInt();
        robot_box_co_state [i] = jsonData["ElectricBoxIO"]["BoxCO"][i].asInt();
        robot_box_ci_state [i] = jsonData["ElectricBoxIO"]["BoxCI"][i].asInt();
    }

    for(int i=0;i<jsonData["EndIO"]["EndDO"].size();i++)
    {
        robot_end_do_state[i] = jsonData["EndIO"]["EndDO"][i].asInt();
        robot_end_di_state [i] = jsonData["EndIO"]["EndDI"][i].asInt();
    }
}

int HRStateClient::ReceiveData()
{
    unsigned char header_bytes[4];
    unsigned char data_len_bytes[8];
    memset(header_bytes, 0, sizeof(header_bytes));
    memset(data_len_bytes, 0, sizeof(data_len_bytes));
    int client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket == -1) {
        std::cerr << "Failed to create socket" << std::endl;
        return -1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, IP.c_str(), &(server_addr.sin_addr));

    if (connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        std::cerr << "Failed to connect server by TCP State" << std::endl;
        close(client_socket);
        return -1;
    }
    while (ThreadSwitch) {
 
        ssize_t ActualDataPacketSize = recv(client_socket, header_bytes, sizeof(header_bytes), 0);
        if (ActualDataPacketSize <= 0) {
                break;
        }
       int res = strncmp((char*)header_bytes, "LTBR", 4);
        if (res == -1) {
            continue;
        }else{
            memset(header_bytes, 0, sizeof(header_bytes));
            ActualDataPacketSize = recv(client_socket, data_len_bytes, sizeof(data_len_bytes), 0);
            int total_size_of_data = ((unsigned char)data_len_bytes[3] << 24) + ((unsigned char)data_len_bytes[2] << 16) + ((unsigned char)data_len_bytes[1] << 8) + (unsigned char)data_len_bytes[0];
            int size_of_data = ((unsigned char)data_len_bytes[7] << 24) + ((unsigned char)data_len_bytes[6] << 16) + ((unsigned char)data_len_bytes[5] << 8) + (unsigned char)data_len_bytes[4];
            if((total_size_of_data-12)!=size_of_data)
            {
                continue;
            }
            unsigned char data_bytes[size_of_data];
            int recv_data_len = 0;
            memset(data_bytes, 0, sizeof(data_bytes));
                while (recv_data_len < size_of_data) {
                int bytesReceived = recv(client_socket, data_bytes+recv_data_len, sizeof(data_bytes)-recv_data_len, 0);
                recv_data_len += bytesReceived;
                if (bytesReceived <= 0 || bytesReceived!=size_of_data) {
                    break;
                }
            }
            std::string str_data(reinterpret_cast<char*>(data_bytes),sizeof(data_bytes));
            OnRecvData(str_data, size_of_data);
        }
       
    }

    close(client_socket);
}