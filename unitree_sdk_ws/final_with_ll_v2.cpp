#include <cmath>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <queue>
#include <stdio.h>
#include <math.h> 

// --- Serial / ESP32 Headers ---
#include <fcntl.h>    // Contains file controls like O_RDWR
#include <errno.h>    // Error integer and strerror() function
#include <termios.h>  // Contains POSIX terminal control definitions
#include <string>
#include <cstring>    // strerror
#include <cerrno>     // errno

// High Level Headers
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

// Low Level Headers
#include <stdint.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

enum test_mode
{
  normal_stand,
  balance_stand,
  velocity_move,
  poke_ground,     // Low Level
  stand_straight,  // Low Level
  stand_down,
  stand_up,
  damp,
  recovery_stand,
  sit,
  rise_sit,
  stop_move = 99,
  // ESP32 Integration
  seeder_next = 100,
  seeder_open = 101,
  seeder_close = 102
};

struct RobotStep {
    test_mode mode;
    double duration;
    double vx = 0.0;
    double vyaw = 0.0;
    double pitch = 0.0;
};

// CRC32 calculation for LowCmd
uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;
    for (unsigned int i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

class Custom
{
public:
  const double MOVE_SPEED = 0.6;    
  const double SEED_DISTANCE = 2.0; 

  bool keep_running = true;
  int last_mode = -1;
  std::queue<RobotStep> sequence;
  double step_start_time = 0.0;
  
  // ESP32 Variables
  int serial_port = -1;

  // Timing and states
  double ct = 0;
  int flag = 0;
  float dt = 0.002; // MUST be 0.002 (500Hz) for Low-Level control stability

  float current_vx = 0.0;
  float current_vyaw = 0.0;

  // Low Level Variables
  double ll_running_time = 0.0;
  double phase = 0.0;
  double captured_stand_pos[12] = {0}; 
  
  double lean_forward[12] = {
      0.00571868, 0.548813, -1.91763, 
     -0.00571868, 0.548813, -1.91763, 
      0.00571868, 0.408813, -1.01763, 
     -0.00571868, 0.408813, -1.01763  
  };
  double lower_body_height[12] = {
      0.00571868, 0.648813, -2.19375, 
     -0.00571868, 0.648813, -2.19375, 
      0.00571868, 0.508813, -1.24375, 
     -0.00571868, 0.508813, -1.24375
  };

  unitree_go::msg::dds_::SportModeState_ high_state{};
  unitree_go::msg::dds_::LowState_ low_state{};
  unitree_go::msg::dds_::LowCmd_ low_cmd{};

  unitree::robot::go2::SportClient sport_client;
  ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> high_suber;
  ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> low_suber;
  ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> low_puber;

  Custom()
  {
    // --- ESP32 BLUETOOTH INIT ---
    serial_port = open("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port < 0) {
        std::cerr << "WARNING: Could not open /dev/rfcomm0. Is the ESP32 bound?" << std::endl;
    } else {
        std::cout << "Successfully connected to Bluetooth ESP32!" << std::endl;
        struct termios tty;
        tcgetattr(serial_port, &tty);
        cfmakeraw(&tty);
        tcsetattr(serial_port, TCSANOW, &tty);
        tcflush(serial_port, TCIOFLUSH); 
    }

    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    double move_duration = SEED_DISTANCE / MOVE_SPEED;

    // --- Sequence Definition ---
    sequence.push({stand_up,      1.0});
    sequence.push({balance_stand, 0.5});
    
    // --- Seed Location 1 ---
    sequence.push({velocity_move, move_duration, MOVE_SPEED, 0.0, 0.0});
    sequence.push({stop_move,     1.0});

    sequence.push({seeder_next,   1.0});  // Send ESP 'NEXT' (Load seed)
    sequence.push({poke_ground,   3.0});  // Low-Level: Lean down
    sequence.push({seeder_open,   1.0});  // Send ESP 'OPEN' (Drop seed)
    sequence.push({stand_straight,3.0});  // Low-Level: Stand up
    sequence.push({seeder_close,  1.0});  // Send ESP 'CLOSE'
    
    sequence.push({recovery_stand, 1.0}); // Give HL balance control back
    sequence.push({balance_stand, 1.0});

    // --- Seed Location 2 ---
    sequence.push({velocity_move, move_duration, MOVE_SPEED, 0.0, 0.0});
    sequence.push({stop_move,     1.0});
    
    sequence.push({seeder_next,   1.0});  // Send ESP 'NEXT' (Load seed)
    sequence.push({poke_ground,   3.0});  // Low-Level: Lean down
    sequence.push({seeder_open,   1.0});  // Send ESP 'OPEN' (Drop seed)
    sequence.push({stand_straight,3.0});  // Low-Level: Stand up
    sequence.push({seeder_close,  1.0});  // Send ESP 'CLOSE'
    
    sequence.push({recovery_stand, 1.0});

    sequence.push({stand_down,    2.0});

    // Initialize Channels
    high_suber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    high_suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

    low_suber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_suber->InitChannel(std::bind(&Custom::LowStateHandler, this, std::placeholders::_1), 1);

    low_puber.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_puber->InitChannel();
  };

  ~Custom() {
      if (serial_port >= 0) {
          close(serial_port);
      }
  }

  void SendMessageToESP32(const std::string& msg) {
      if (serial_port < 0) {
          std::cerr << "Cannot send '" << msg << "' - Bluetooth not connected." << std::endl;
          return;
      }
      char trash_buffer[256];
      read(serial_port, trash_buffer, sizeof(trash_buffer)); 
      
      std::string payload = msg + "\n"; 
      int bytes_written = write(serial_port, payload.c_str(), payload.length());
      if (bytes_written < 0) {
          std::cerr << "Failed to write to Bluetooth port." << std::endl;
      }
  }

  void InitLowCmd()
  {
      low_cmd.head()[0] = 0xFE;
      low_cmd.head()[1] = 0xEF;
      low_cmd.level_flag() = 0xFF;
      low_cmd.gpio() = 0;
      for (int i = 0; i < 20; i++)
      {
          low_cmd.motor_cmd()[i].mode() = 0x01; 
          low_cmd.motor_cmd()[i].q() = PosStopF;
          low_cmd.motor_cmd()[i].kp() = 0;
          low_cmd.motor_cmd()[i].dq() = VelStopF;
          low_cmd.motor_cmd()[i].kd() = 0;
          low_cmd.motor_cmd()[i].tau() = 0;
      }
  }

  void WriteLowCmd(double* target_q)
  {
      for (int i = 0; i < 12; i++) {
          low_cmd.motor_cmd()[i].q() = target_q[i];
          low_cmd.motor_cmd()[i].dq() = 0;
          low_cmd.motor_cmd()[i].kp() = 50.0;
          low_cmd.motor_cmd()[i].kd() = 3.5;
          low_cmd.motor_cmd()[i].tau() = 0;
      }
      low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
      low_puber->Write(low_cmd);
  }

  void RobotControl()
  {
    ct += dt;

    if (sequence.empty()) {
        keep_running = false;
        return;
    }

    RobotStep& current = sequence.front();
    int TEST_MODE = current.mode;
    current_vx = current.vx;
    current_vyaw = current.vyaw;

    double elapsed = ct - step_start_time;
    if (elapsed >= current.duration) {
        sequence.pop();
        step_start_time = ct;
        last_mode = -1; 
        
        if (sequence.empty()) {
            std::cout << "Sequence finished." << std::endl;
            keep_running = false;
            return;
        }
        return; 
    }

    // STATE MACHINE
    switch (TEST_MODE)
    {
    case velocity_move: 
      sport_client.Move(current_vx, 0, current_vyaw);
      break;

    case stop_move: 
      sport_client.StopMove();
      break;

    // --- ESP32 Commands ---
    case seeder_next:
      sport_client.BalanceStand(); // Keep holding balance 
      if (TEST_MODE != last_mode) {
          std::cout << ">>> SENDING 'NEXT' TO ESP32 <<<" << std::endl;
          SendMessageToESP32("NEXT");
      }
      break;

    case seeder_open:
      // When OPEN is sent, we are currently in the Low-Level poking stance.
      // We must KEEP sending the LowCmd so the robot doesn't collapse!
      WriteLowCmd(lower_body_height); 
      if (TEST_MODE != last_mode) {
          std::cout << ">>> SENDING 'OPEN' TO ESP32 <<<" << std::endl;
          SendMessageToESP32("OPEN");
      }
      break;

    case seeder_close:
      sport_client.BalanceStand(); // Back to HL balance
      if (TEST_MODE != last_mode) {
          std::cout << ">>> SENDING 'CLOSE' TO ESP32 <<<" << std::endl;
          SendMessageToESP32("CLOSE");
      }
      break;

    // --- Low-Level Maneuvers ---
    case poke_ground:
      if (TEST_MODE != last_mode) {
          sport_client.Damp(); 
          InitLowCmd();        
          ll_running_time = 0.0;
          for(int i=0; i<12; i++) {
              captured_stand_pos[i] = low_state.motor_state()[i].q();
          }
      }
      
      ll_running_time += dt;
      double current_target[12];

      if (ll_running_time < 1.5) {
          phase = tanh(ll_running_time / 0.5); 
          for (int i = 0; i < 12; i++) {
              current_target[i] = phase * lean_forward[i] + (1 - phase) * captured_stand_pos[i];
          }
      } else {
          phase = tanh((ll_running_time - 1.5) / 0.5);
          for (int i = 0; i < 12; i++) {
              current_target[i] = phase * lower_body_height[i] + (1 - phase) * lean_forward[i];
          }
      }
      WriteLowCmd(current_target);
      break;

    case stand_straight:
      if (TEST_MODE != last_mode) {
          ll_running_time = 0.0;
      }
      
      ll_running_time += dt;
      double recovery_target[12];
      
      phase = tanh(ll_running_time / 0.8);
      for (int i = 0; i < 12; i++) {
          recovery_target[i] = phase * captured_stand_pos[i] + (1 - phase) * lower_body_height[i];
      }
      WriteLowCmd(recovery_target);
      break;

    // Standard High Level fallbacks
    case balance_stand: sport_client.BalanceStand(); break;
    case stand_down: if(TEST_MODE!=last_mode) sport_client.StandDown(); break;
    case stand_up: if(TEST_MODE!=last_mode) sport_client.StandUp(); break;
    case recovery_stand: if(TEST_MODE!=last_mode) sport_client.RecoveryStand(); break;
    default: sport_client.StopMove();
    }

    last_mode = TEST_MODE;
  };

  void HighStateHandler(const void *message) {
    high_state = *(unitree_go::msg::dds_::SportModeState_ *)message;
  };
  
  void LowStateHandler(const void *message) {
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
  };
};

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface (e.g., eth0 or wlan0)" << std::endl;
    exit(-1);
  }

  ChannelFactory::Instance()->Init(0, argv[1]);
  Custom custom;
  sleep(1);

  ThreadPtr threadPtr = CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::RobotControl, &custom));

  while (custom.keep_running) {
    sleep(1);
  }

  std::cout << "Script sequence complete. Shutting down cleanly..." << std::endl;
  return 0;
}