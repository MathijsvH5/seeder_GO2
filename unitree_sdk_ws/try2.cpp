#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp> // <-- NEW HEADER

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom
{
public:
    explicit Custom(){}
    ~Custom(){}

    void Init();
    void Start();
    
private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();

private:
    float Kp = 60.0;
    float Kd = 5.0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01


    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    unitree_go::msg::dds_::LowState_ low_state{};  // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;

    float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

    float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                              0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    float _targetPos_3[12] = {
        0.00571868, 0.548813, -1.91763, //FR
       -0.00571868, 0.548813, -1.91763, //FL
        0.00571868, 0.408813, -1.01763, //RR
       -0.00571868, 0.408813, -1.01763 };

    float _targetPos_4[12] = {
        0.00571868, 0.648813, -2.19375, 
       -0.00571868, 0.648813, -2.19375, 
        0.00571868, 0.508813, -1.24375, 
       -0.00571868, 0.508813, -1.24375};

    float _startPos[12];
    float _duration_1 = 500;   
    float _duration_2 = 500; 
    float _duration_3 = 500;   
    float _duration_4 = 500;   
    float _percent_1 = 0;    
    float _percent_2 = 0;    
    float _percent_3 = 0;    
    float _percent_4 = 0;    

    bool firstRun = true;
    bool done = false;
};

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Custom::Init()
{
    InitLowCmd();

    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    // --- THE EXPLICIT SNIPER METHOD ---
    RobotStateClient rsc;
    rsc.SetTimeout(10.0f); 
    rsc.Init();
    
    std::cout << "Waiting 2 seconds for DDS network..." << std::endl;
    sleep(2); 

    int32_t status = 0; // A variable to hold the output status
    
    std::cout << "Target 1: Assassinating 'mcf' service..." << std::endl;
    // 0 means turn OFF
    int32_t ret_mcf = rsc.ServiceSwitch("mcf", 0, status);
    std::cout << "mcf kill result: " << ret_mcf << std::endl;

    std::cout << "Target 2: Assassinating 'sport_mode' service..." << std::endl;
    int32_t ret_sport = rsc.ServiceSwitch("sport_mode", 0, status);
    std::cout << "sport_mode kill result: " << ret_sport << std::endl;

    sleep(2); // Give the firmware a second to process both commands
    std::cout << "All balancing AI should now be completely dead." << std::endl;
}

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    for(int i=0; i<20; i++) {
        low_cmd.motor_cmd()[i].mode() = 0x01;  
        low_cmd.motor_cmd()[i].q() = PosStopF;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].dq() = VelStopF;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}

void Custom::Start()
{
    std::cout << "Waiting for VALID sensor data from the robot..." << std::endl;
    
    while (true) {
        if (std::abs(low_state.motor_state()[0].q()) > 0.001 || 
            std::abs(low_state.motor_state()[1].q()) > 0.001) {
            std::cout << ">>> Real sensor data confirmed! Starting Sequence. <<<" << std::endl;
            break; 
        }
        usleep(10000); 
    }

    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &Custom::LowCmdWrite, this);
}

void Custom::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void Custom::LowCmdWrite()
{

    motiontime++;
    if(motiontime>=500)
    {
        if(firstRun)
        {
            for(int i = 0; i < 12; i++)
            {
                _startPos[i] = low_state.motor_state()[i].q();
            }
            firstRun = false;
        }

        _percent_1 += (float)1 / _duration_1;
        _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
        if (_percent_1 < 1)
        {
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_1) * _startPos[j] + _percent_1 * _targetPos_1[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        
        }
        if ((_percent_1 == 1)&&(_percent_2 < 1))
        {
            _percent_2 += (float)1 / _duration_2;
            _percent_2 = _percent_2 > 1 ? 1 : _percent_2;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_2) * _targetPos_1[j] + _percent_2 * _targetPos_2[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }

        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3<1))
        {
            _percent_3 += (float)1 / _duration_3;
            _percent_3 = _percent_3 > 1 ? 1 : _percent_3;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() =  (1 - _percent_3) * _targetPos_2[j] + _percent_3 * _targetPos_3[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }
        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3==1)&&((_percent_4<=1)))
        {
            _percent_4 += (float)1 / _duration_4;
            _percent_4 = _percent_4 > 1 ? 1 : _percent_4;
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_4) * _targetPos_3[j] + _percent_4 * _targetPos_4[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }
        low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
        lowcmd_publisher->Write(low_cmd);
    }
   
}


int main(int argc, const char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface (e.g. eno1)" << std::endl;
        exit(-1); 
    }

    std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // --- INJECTING YOUR UNICAST XML FOR SENSOR DATA ---
    std::string interface_name = argv[1];
    std::string custom_xml = 
        "<CycloneDDS><Domain><General><Interfaces>"
        "<NetworkInterface name=\"" + interface_name + "\"/>"
        "</Interfaces></General><Discovery><Peers>"
        "<Peer address=\"192.168.12.1\"/>"
        "</Peers></Discovery></Domain></CycloneDDS>";

    setenv("CYCLONEDDS_URI", custom_xml.c_str(), 1);
    ChannelFactory::Instance()->Init(0, "");

    Custom custom;
    custom.Init();
    custom.Start();
    
    while (1) {
        sleep(10);
    }
    return 0;
}