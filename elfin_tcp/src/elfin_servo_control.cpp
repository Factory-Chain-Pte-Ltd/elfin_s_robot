#include "elfin_tcp/elfin_servo_control.h"

HRServoClient::HRServoClient()
{
    bServoConnect = false;
    hScoket = -1;
    return;
}

HRServoClient::~HRServoClient()
{
    if(bServoConnect)
    {
        disconnect();
    }
}

int HRServoClient::connectToServo(const string servo_ip, const int servo_port)
{
    std::cout<<"Elfin Servo Control Connect by IP: "<<servo_ip<<","<<"port: "<<servo_port<<std::endl;
    bServoConnect = false;
    hScoket = socket(AF_INET, SOCK_STREAM, 0);
    if(hScoket == -1)
    {
        close(hScoket);
        std::cerr<<"Failed to create socket..."<<std::endl;
        return -1;
    }
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(servo_port);
    server_addr.sin_addr.s_addr = inet_addr(servo_ip.c_str());
    if(connect(hScoket, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
        std::cout<<"Failed to connect to server..."<<std::endl;
        close(hScoket);
        return -1;
    }
    bool bKeepAlive = true;
    setsockopt(hScoket, SOL_SOCKET, SO_KEEPALIVE, (char*)&bKeepAlive, sizeof(bKeepAlive));
    bServoConnect = true;
    return 0;
}

int HRServoClient::disconnect()
{
    pthread_mutex_lock(&ServosocketMutex);
    if(bServoConnect)
    {
        close(hScoket);
    }
    hScoket = -1;
    bServoConnect = false;
    pthread_mutex_unlock(&ServosocketMutex);
    return 0;
}

int HRServoClient::StartServo(double servotime, double lookheadtime)
{
    string cmd_str = "StartServo,0,";
    cmd_str.append(to_string(servotime));
    cmd_str.append(",");
    cmd_str.append(to_string(lookheadTime));
    cmd_str.append(",;");
    int nRet = send(hScoket, cmd_str.c_str(),cmd_str.size(), 0);
    return nRet;
}

int HRServoClient::pushServoJ(double j1, double j2, double j3, double j4, double j5, double j6)
{

    string cmd_str = "PushServoJ,0,";
    cmd_str.append(to_string(j1));
    cmd_str.append(",");
    cmd_str.append(to_string(j2));
    cmd_str.append(",");
    cmd_str.append(to_string(j3));
    cmd_str.append(",");
    cmd_str.append(to_string(j4));
    cmd_str.append(",");
    cmd_str.append(to_string(j5));
    cmd_str.append(",");
    cmd_str.append(to_string(j6));
    cmd_str.append(",;");
    int nRet = send(hScoket, cmd_str.c_str(), cmd_str.size(), 0);
    return nRet;
}