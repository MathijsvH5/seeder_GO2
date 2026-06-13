#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <cstdlib>

// High Level & State Clients
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

// Low Level Clients
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// --- The Master Sequence Timeline ---
enum SequenceState {
    HL_STAND_UP,
    ASSASSINATE_AI,
    LL_POKE_SEQUENCE,
    LL_LET_GO,
    RESURRECT_AI,
    HL_RECOVERY,
    HL_STAND_DOWN,
    DONE
};

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0, data = 0, CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;
    for (unsigned int i = 0; i < len; i++) {
        xbit = 1 << 31; data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) { CRC32 <<= 1; CRC32 ^= dwPolynomial; }
            else { CRC32 <<= 1; }
            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

class Custom
{
public:
    bool keep_running = true;
    SequenceState current_state = HL_STAND_UP;
    SequenceState last_state = DONE; 
    
    double ct = 0.0;
    float dt = 0.002; // 500Hz loop

    // Clients
    SportClient sport_client;
    RobotStateClient rsc;
    
    // Channels
    unitree_go::msg::dds_::LowCmd_ low_cmd{};      
    unitree_go::msg::dds_::LowState_ low_state{};  
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ThreadPtr controlThreadPtr;

    // Low Level Tuning
    float Kp = 60.0;
    float Kd = 5.0;

    // Poses
    float _startPos[12]; // Will hold the exact standing pose
    float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
    float _targetPos_3[12] = {0.00571868, 0.548813, -1.91763, -0.00571868, 0.548813, -1.91763, 0.00571868, 0.408813, -1.01763, -0.00571868, 0.408813, -1.01763};
    float _targetPos_4[12] = {0.00571868, 0.648813, -2.19375, -0.00571868, 0.648813, -2.19375, 0.00571868, 0.508813, -1.24375, -0.00571868, 0.508813, -1.24375};

    Custom()
    {
        // 1. Init High Level Clients
        sport_client.SetTimeout(10.0f);
        sport_client.Init();
        rsc.SetTimeout(10.0f);
        rsc.Init();

        // 2. Init Low Level Channels
        InitLowCmd();
        lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        lowcmd_publisher->InitChannel();
        lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);
    }

    void InitLowCmd()
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

    void Start()
    {
        std::cout << "Waiting for VALID sensor data from the robot..." << std::endl;
        while (true) {
            if (std::abs(low_state.motor_state()[0].q()) > 0.001 || std::abs(low_state.motor_state()[1].q()) > 0.001) {
                std::cout << ">>> Ready! Starting Master Sequence in 2 seconds... <<<" << std::endl;
                break; 
            }
            usleep(10000); 
        }
        sleep(2); // Final breath before execution

        // Start the master 500Hz loop
        controlThreadPtr = CreateRecurrentThreadEx("master_control", UT_CPU_ID_NONE, 2000, &Custom::MasterControlLoop, this);
    }

    void LowStateMessageHandler(const void* message)
    {
        low_state = *(unitree_go::msg::dds_::LowState_*)message;
    }

    void WriteLowCmdTarget(float* start, float* end, float percent)
    {
        for (int j = 0; j < 12; j++) {
            low_cmd.motor_cmd()[j].q() = (1 - percent) * start[j] + percent * end[j];
            low_cmd.motor_cmd()[j].dq() = 0;
            low_cmd.motor_cmd()[j].kp() = Kp;
            low_cmd.motor_cmd()[j].kd() = Kd;
            low_cmd.motor_cmd()[j].tau() = 0;
        }
        low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
        lowcmd_publisher->Write(low_cmd);
    }

    void MasterControlLoop()
    {
        ct += dt; // Tick the clock

        // Print timeline every 1 second
        if (std::fmod(ct, 1.0) < 0.002) { 
            std::cout << "Time: " << ct << "s | State: " << current_state << std::endl; 
        }

        // --- 1. DETERMINE CURRENT STATE BASED ON TIME ---
        if      (ct < 3.0)  current_state = HL_STAND_UP;       // 0-3s
        else if (ct < 5.0)  current_state = ASSASSINATE_AI;    // 3-5s
        else if (ct < 11.0) current_state = LL_POKE_SEQUENCE;  // 5-11s (Added time to lie down)
        else if (ct < 11.5) current_state = LL_LET_GO;         // 11-11.5s (Turn motors off)
        else if (ct < 16.5) current_state = RESURRECT_AI;      // 11.5-16.5s (5 SECONDS FOR AI BOOT)
        else if (ct < 19.0) current_state = HL_RECOVERY;       // 16.5-19s (AI Stands Up)
        else if (ct < 22.0) current_state = HL_STAND_DOWN;     // 19-22s (AI Lies Down)
        else                current_state = DONE;

        // --- 2. EXECUTE LOGIC ---
switch (current_state)
        {
        case HL_STAND_UP:
            if (current_state != last_state) sport_client.StandUp();
            break;

        case ASSASSINATE_AI:
            if (current_state != last_state) {
                std::cout << "Killing AI to take over..." << std::endl;
                int32_t status = 0;
                rsc.ServiceSwitch("mcf", 0, status);
                rsc.ServiceSwitch("sport_mode", 0, status);
                
                for(int i = 0; i < 12; i++) {
                    _startPos[i] = low_state.motor_state()[i].q();
                }
            }
            WriteLowCmdTarget(_startPos, _startPos, 1.0);
            break;

        case LL_POKE_SEQUENCE:
            {
                double ll_time = ct - 5.0; // Starts at 0.0 

                if (ll_time < 1.0) {        // 0-1s: Stand -> Lean
                    WriteLowCmdTarget(_startPos, _targetPos_3, ll_time / 1.0);
                } 
                else if (ll_time < 2.0) {   // 1-2s: Lean -> Poke
                    WriteLowCmdTarget(_targetPos_3, _targetPos_4, (ll_time - 1.0) / 1.0);
                }
                else if (ll_time < 3.0) {   // 2-3s: Poke -> Stand
                    WriteLowCmdTarget(_targetPos_4, _targetPos_1, (ll_time - 2.0) / 1.0);
                }
                else {                      // 5-6s: Hold Lie Down pose
                    WriteLowCmdTarget(_targetPos_1, _targetPos_1, 1.0);
                }
            }
            break;

        case LL_LET_GO:
            // Explicitly set torque and stiffness to 0 so the motors completely relax.
            for(int j=0; j<12; j++){
                low_cmd.motor_cmd()[j].mode() = 0x01;
                low_cmd.motor_cmd()[j].q() = PosStopF;
                low_cmd.motor_cmd()[j].kp() = 0;
                low_cmd.motor_cmd()[j].dq() = VelStopF;
                low_cmd.motor_cmd()[j].kd() = 0;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
            low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
            lowcmd_publisher->Write(low_cmd);
            break;

        case RESURRECT_AI:
            if (current_state != last_state) {
                std::cout << "Robot is resting safely. Waking AI... Waiting 5 seconds." << std::endl;
                int32_t status = 0;
                rsc.ServiceSwitch("mcf", 1, status);         
                rsc.ServiceSwitch("sport_mode", 1, status);  
            }
            // DO NOTHING ELSE. The motors are off, and the AI is booting.
            break;

        case HL_RECOVERY:
            if (current_state != last_state) {
                std::cout << "AI Active. Sending Recovery Stand." << std::endl;
                sport_client.RecoveryStand();
            }
            break;

        case HL_STAND_DOWN:
            if (current_state != last_state) {
                std::cout << "Laying down gracefully." << std::endl;
                sport_client.StandDown();
            }
            break;

        case DONE:
            keep_running = false;
            break;
        }

        last_state = current_state;
    }
};

int main(int argc, const char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface (e.g. eno1)" << std::endl;
        exit(-1); 
    }

    std::cout << "--- MASTER SEQUENCE INITIALIZING ---" << std::endl;
    std::cout << "Make sure the robot is on the floor with space to stand up." << std::endl;
    std::cout << "Press Enter to execute sequence..." << std::endl;
    std::cin.ignore();

    // Inject Unicast XML
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
    custom.Start();
    
    while (custom.keep_running) {
        sleep(1);
    }
    
    std::cout << "Sequence Complete. Exiting Safely." << std::endl;
    return 0;
}