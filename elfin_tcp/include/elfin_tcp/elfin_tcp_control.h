#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstring>
#include <vector>
#include <atomic>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <string>
#include <chrono>
#include <mutex>

using namespace std;

class HRControlClient
{
    public:
        HRControlClient();
        ~HRControlClient();

    public:
        int connect_state;
        int security_level;
        int robot_state;
        int error_code;
        int cmd_resutl;
        int robot_do_state[8];
        int robot_co_state[8];
        int robot_endDO_state[3];

    public:
        std::string IP;
        int control_port = 10003;
        int enable_robot(string& cmd_res);
        int disable_robot(string& cmd_res);
        int reset_robot(string& cmd_res);
        int set_override(double override,string& cmd_res);
        int open_freedriver(string& cmd_res);
        int close_freedriver(string& cmd_res);
        int set_security_level(int level,string& cmd_res);
        int set_robot_do(int do_bit, int do_state, string& cmd_res);
        int set_robot_co(int do_bit, int do_state, string& cmd_res);
        int set_end_do(int EndDO_bit, int EndDO_State, string& cmd_res);
        int connectToRobot(const string robot_ip,const int robot_prot);
        int disConnect();
        int SendMsg(string& strSendMsg, int nSize, string& strRecv, bool bwaitRel, int nWaitSec);
        int RecvMsg(string& strRecv, int nWaitSec, string strEnd);
        int getCmdRes(string strRecv);
        int StartServo(double servotime, double lookheadtime);
        int pushServoJ(double j1, double j2, double j3, double j4, double j5, double j6);
        int StopMove();
        int start_long_motion();
        int longMoveJ();
        int longMoveL();
    protected:
        char m_szRecv[4096];
        bool m_bConnect;
        int m_hSocket;
        pthread_mutex_t socketMutex = PTHREAD_MUTEX_INITIALIZER;
        std::mutex mtx;
};