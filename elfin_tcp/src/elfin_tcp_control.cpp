#include "elfin_tcp/elfin_tcp_control.h"

HRControlClient::HRControlClient()
{
    m_bConnect = false;
    m_hSocket = -1;
    return;
}

HRControlClient::~HRControlClient()
{
    if(m_bConnect)
    {
        disConnect();
    }
}

int HRControlClient::connectToRobot(const string robot_ip, const int robot_port)
{
    std::cout<<"Elfin TCP Control Connect by IP: "<<robot_ip<<","<<"port: "<<robot_port<<std::endl;
    memset(m_szRecv, 0, sizeof(m_szRecv));
    m_bConnect = false; 
    m_hSocket = socket(AF_INET, SOCK_STREAM, 0);
    if(m_hSocket == -1)
    {
        close(m_hSocket);
        std::cerr<<"Failed to create socket"<<std::endl;
        return -1;
    }
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(robot_port);
    server_addr.sin_addr.s_addr = inet_addr(robot_ip.c_str());
    if(connect(m_hSocket, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
        std::cout<<"Failed to connect  server by TCP Control"<<std::endl;
        close(m_hSocket);
        return -1;
    }
    bool bKeepAlive = true;
    setsockopt(m_hSocket, SOL_SOCKET, SO_KEEPALIVE, (char*)&bKeepAlive, sizeof(bKeepAlive));
    m_bConnect = true;
    return 0;
}

int HRControlClient::disConnect()
{
    pthread_mutex_lock(&socketMutex);
    if(m_bConnect)
    {
        close(m_hSocket);
    }
    m_hSocket = -1;
    m_bConnect = false;
    pthread_mutex_unlock(&socketMutex);
    return 0;
}

int HRControlClient::SendMsg(string& strSendMsg, int nSize, string& strRecv, bool bwaitRel, int nWaitSec)
{
    if(strSendMsg.size() ==0)
    {
        std::cout<<"SendMsg Empty!!!"<<std::endl;
        return 0;
    }
    mtx.lock();
    int nRet = send(m_hSocket, strSendMsg.c_str(), nSize, 0);
    if(nRet != nSize)
    {
        return errno;
    }
    if(!bwaitRel)
    {
        return 0;
    }
    return RecvMsg(strRecv, nWaitSec, ",;");
}

int HRControlClient::RecvMsg(string& strRecv, int nWaitSec, string strEnd)
{
    int nRet = 0;
    int nCNT = 0;
    if(nWaitSec <0)
    {
        nWaitSec = 0;
    }
    if(nWaitSec >30)
    {
        nWaitSec = 30;
    }

    do
    {
        struct  timeval timeout = {nWaitSec, 1000*100};
        memset(m_szRecv,0,sizeof(m_szRecv));
        // mtx.lock();
        nRet = recv(m_hSocket, m_szRecv, sizeof(m_szRecv) -1, 0);
        mtx.unlock();
        if(nRet >0)
        {
            string csRecv(m_szRecv, nRet);
            strRecv.append(csRecv);
            if(strRecv.rfind(strEnd)!=string::npos)
            {   
                break;
            }
            else if(nCNT>=5)
            {
                return -1;
            }
        }
        else if(nRet == 0)
        {
            break;
        }
        else
        {
            m_bConnect = false;
            return errno;
        }
    } while (true);
    return 0;
}

int HRControlClient::getCmdRes(string strRecv)
{
    vector<string> cmd_res;
    stringstream sstr(strRecv);
    string res;
    while (getline(sstr, res, ','))
    {
        cmd_res.push_back(res);
    }
    if(cmd_res[1] == "OK")
    {
        return 0;
    }
    else if(cmd_res[1] == "Fail")
    {
        return stoi(cmd_res[2]);
    }
    return 1;
    
}

int HRControlClient::enable_robot(string& cmd_res)
{
    if(m_bConnect == false)
    {
        return -1;
    }
    string strRet;
    string cmd_str = "GrpEnable,0,;";
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    cmd_res = strRet;
    return getCmdRes(strRet);
}

int HRControlClient::disable_robot(string& cmd_res)
{
    if(m_bConnect == false)
    {
        return -1;
    }
    string strRet;
    string cmd_str = "GrpDisable,0,;";
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    cmd_res = strRet;
    return getCmdRes(strRet);
}

int HRControlClient::reset_robot(string& cmd_res)
{
    if(m_bConnect == false)
    {
        return -1;
    }
    string strRet;
    string cmd_str = "GrpReset,0,;";
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    cmd_res = strRet;
    return getCmdRes(strRet);
}

int HRControlClient::set_override(double override,string& cmd_res)
{
    if(m_bConnect == false)
    {
        return -1;
    }
    string strRet;
    string cmd_str = "SetOverride,0,";
    cmd_str.append(to_string(override));
    cmd_str.append(",;");
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    cmd_res = strRet;
    return getCmdRes(strRet);
}   

int HRControlClient::open_freedriver(string& cmd_res)
{
    if(m_bConnect == false)
    {
        return -1;
    }
    string strRet;
    string cmd_str = "GrpOpenFreeDriver,0,;";
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    cmd_res = strRet;
    return getCmdRes(strRet);
}

int HRControlClient::close_freedriver(string& cmd_res)
{
    if(m_bConnect == false)
    {
        return -1;
    }
    string strRet;
    string cmd_str = "GrpCloseFreeDriver,0,;";
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    cmd_res = strRet;
    return getCmdRes(strRet);
}

int HRControlClient::set_security_level(int level,string& cmd_res)
{
    if(m_bConnect == false)
    {
        return -1;
    }
    string strRet;
    string cmd_str = "SetCollideLevel,0,";
    cmd_str.append(to_string(level));
    cmd_str.append(",;");
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    cmd_res = strRet;
    return getCmdRes(strRet);
}

int HRControlClient::set_robot_do(int do_bit, int do_state, string& cmd_res)
{
    if(m_bConnect == false)
    {
        return -1;
    }
    string strRet;
    string cmd_str = "SetBoxDO,";
    cmd_str.append(to_string(do_bit));
    cmd_str.append(",");
    cmd_str.append(to_string(do_state));
    cmd_str.append(",;");
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    cmd_res = strRet;
    return getCmdRes(strRet);
}

int HRControlClient::set_robot_co(int do_bit, int do_state, string& cmd_res)
{
    if(m_bConnect == false)
    {
        return -1;
    }
    string strRet;
    string cmd_str = "SetBoxCO,";
    cmd_str.append(to_string(do_bit));
    cmd_str.append(",");
    cmd_str.append(to_string(do_state));
    cmd_str.append(",;");
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    cmd_res = strRet;
    return getCmdRes(strRet);
}

int HRControlClient::set_end_do(int EndDO_bit, int EndDO_State, string& cmd_res)
{
    string strRet;
    string cmd_str = "SetEndDO,0,";
    cmd_str.append(to_string(EndDO_bit));
    cmd_str.append(",");
    cmd_str.append(to_string(EndDO_State));
    cmd_str.append(",;");
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    cmd_res = strRet;
    return getCmdRes(strRet);
}

int HRControlClient::StartServo(double servotime, double lookheadtime)
{
    string strRet;
    string cmd_str = "StartServo,0,";
    cmd_str.append(to_string(servotime));
    cmd_str.append(",");
    cmd_str.append(to_string(lookheadtime));
    cmd_str.append(",;");
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    return getCmdRes(strRet);
}

int HRControlClient::pushServoJ(double j1, double j2, double j3, double j4, double j5, double j6)
{
    string strRet;
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
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    return getCmdRes(strRet);
}
int HRControlClient::StopMove()
{
    if(m_bConnect == false)
    {
        return -1;
    }
    string strRet;
    string cmd_str = "GrpStop,0,;";
    int nRet = SendMsg(cmd_str, cmd_str.size(), strRet, true, 0);
    return getCmdRes(strRet);
}
int HRControlClient::start_long_motion()
{

    return 0;
}


int HRControlClient::longMoveJ()
{

    return 0;
}

int HRControlClient::longMoveL()
{
    return 0;
}