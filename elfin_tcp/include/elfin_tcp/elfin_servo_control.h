#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstring>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <string>
#include <chrono>

using namespace std;

class HRServoClient
{
    public:
        HRServoClient();
        ~HRServoClient();
    
    public:
        double servoTime = 0.0;
        double lookheadTime = 0.0;
        std::string IP;
        int servo_port = 8892;
        int connectToServo(const string servo_ip, const int servo_port);
        int disconnect();
        int StartServo(double servotime, double lookheadtime);
        int pushServoJ(double j1, double j2, double j3, double j4, double j5, double j6);
    protected:
        bool bServoConnect;
        int hScoket;    
        pthread_mutex_t ServosocketMutex = PTHREAD_MUTEX_INITIALIZER;    
};  