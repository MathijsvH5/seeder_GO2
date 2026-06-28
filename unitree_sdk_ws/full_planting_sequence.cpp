/********************************************************************** 
 * Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved. 
 ***********************************************************************/

#include <cmath> 
#include <iostream> 
#include <vector> 
#include <unistd.h> 
#include <queue>  

// Bluetooth and Serial Headers
#include <fcntl.h>    
#include <errno.h>    
#include <termios.h>  
#include <unistd.h>   
#include <string>
#include <cstring>    
#include <cerrno>     

#include <unitree/robot/go2/sport/sport_client.hpp> 
#include <unitree/robot/channel/channel_subscriber.hpp> 
#include <unitree/idl/go2/SportModeState_.hpp> 

#define TOPIC_HIGHSTATE "rt/sportmodestate" 

using namespace unitree::common; 

enum test_mode 
{ 
  /*---Basic motion---*/ 
  normal_stand, 
  balance_stand, 
  velocity_move, 
  pitch_control,
  height_control, 
  stand_down, 
  stand_up, 
  damp, 
  recovery_stand, 
  /*---Special motion ---*/ 
  sit, 
  rise_sit, 
  set_walk,
  stop_move = 99,
  // ESP32 Integration
  seeder_next = 100,
  seeder_open = 101,
  seeder_close = 102
}; 

// Define the structure for each action in our sequence 
struct RobotStep { 
    test_mode mode; 
    double duration; 
    double vx = 0.0; 
    double vyaw = 0.0; 
    double pitch = 0.0; 
    double yaw = 0.0;
}; 

class Custom 
{ 
public: 
  // Constants
  const double MOVE_SPEED = 0.6;    // 0.6 m/s  
  const double SEED_DISTANCE = 1.5;

  // Sequence control
  bool keep_running = true;  
  int last_mode = -1;        
  bool has_sent_message = false; // Tracker for our robust ESP logic
  std::queue<RobotStep> sequence; 
  double step_start_time = 0.0; 

  int serial_port = -1; // Holds the file descriptor for Bluetooth

  Custom() 
  { 
    // --- OPEN THE BLUETOOTH PORT ---
    serial_port = open("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port < 0) {
        std::cerr << "WARNING: Could not open /dev/rfcomm0. Is the ESP32 bound?" << std::endl;
    } else {
        std::cout << "Successfully connected to Bluetooth ESP32!" << std::endl;
        
        // Configure port for raw text output
        struct termios tty;
        tcgetattr(serial_port, &tty);
        cfmakeraw(&tty);
        tcsetattr(serial_port, TCSANOW, &tty);
        tcflush(serial_port, TCIOFLUSH); // Clears the pipes
    }

    sport_client.SetTimeout(10.0f); 
    sport_client.Init(); 

    double move_duration = SEED_DISTANCE / MOVE_SPEED;   

    // Initial Wakeup & Stand 
    sequence.push({stand_up,      2.5}); // Safe stand up duration!
    sequence.push({balance_stand, 0.5}); 
    sequence.push({set_walk,      0.5});
     
    for (int i = 0; i < 2; ++i) {
      // Walk to seed location
      sequence.push({velocity_move, move_duration, MOVE_SPEED, 0.0, 0.0, 0.0}); 
      sequence.push({stop_move,     1.0}); 

      // 1. Send 'NEXT' while in neutral stance
      sequence.push({seeder_next,   1.0, 0.0, 0.0, 0.0, 0.0});

      // Position 1: Lean forward and yaw left
      sequence.push({pitch_control, 0.5, 0.0, 0.0, 0.0, -0.45});
      sequence.push({pitch_control, 1.0, 0.0, 0.0, 0.2, -0.45}); 
      
      // 2. Open seeder (maintaining pitch 0.2 and yaw -0.45)
      sequence.push({seeder_open,   1.0, 0.0, 0.0, 0.2, -0.45});

      // Return to neutral stance
      sequence.push({pitch_control, 1.0, 0.0, 0.0, 0.0, 0.0}); 

      // 3. Send CLOSE then NEXT while in neutral stance
      sequence.push({seeder_close,  1.0, 0.0, 0.0, 0.0, 0.0});
      sequence.push({seeder_next,   1.0, 0.0, 0.0, 0.0, 0.0});

      // Position 2: Lean forward and yaw right
      sequence.push({pitch_control, 0.5, 0.0, 0.0, 0.0, 0.45});
      sequence.push({pitch_control, 1.0, 0.0, 0.0, 0.2, 0.45}); 

      // 4. Open seeder (maintaining pitch 0.2 and yaw 0.45)
      sequence.push({seeder_open,   1.0, 0.0, 0.0, 0.2, 0.45});

      // Return to neutral stance
      sequence.push({pitch_control, 1.0, 0.0, 0.0, 0.0, 0.0}); 

      // 5. Send CLOSE before moving on
      sequence.push({seeder_close,  1.0, 0.0, 0.0, 0.0, 0.0});
      
      sequence.push({set_walk,      0.5});
    }

    // Finish and keep standing
    sequence.push({stand_up, 3.5}); 

    // Initialize subscriber for robot state 
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE)); 
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1); 
  }; 

  ~Custom() {
      if (serial_port >= 0) {
          close(serial_port);
      }
  }

  bool SendMessageToESP32(const std::string& msg) {
      if (serial_port < 0) {
          std::cerr << "Cannot send '" << msg << "' - Bluetooth not connected." << std::endl;
          return false;
      }
      
      // 1. Throw away any pending messages from the ESP32 to clear the buffer
      char trash_buffer[256];
      while (read(serial_port, trash_buffer, sizeof(trash_buffer)) > 0) {
          // Empty out the buffer
      }
      
      // 2. Send the new message
      std::string payload = msg + "\n"; 
      int bytes_written = write(serial_port, payload.c_str(), payload.length());
      
      if (bytes_written < 0) {
          static bool error_printed = false;
          if (!error_printed) {
              std::cerr << "[BLUETOOTH ERROR] Failed to write to port. Kernel says: " << strerror(errno) << std::endl;
              error_printed = true;
          }
          return false;
      }
      return true;
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
    current_pitch = current.pitch; 
    current_yaw = current.yaw;

    double elapsed = ct - step_start_time; 
    if (elapsed >= current.duration) { 
        std::cout << "Time: " << ct << "s | Completed mode: " << TEST_MODE << std::endl; 
        sequence.pop();              
        step_start_time = ct;       
        has_sent_message = false; // Reset the flag for the next step!
         
        if (sequence.empty()) { 
            std::cout << "Sequence finished." << std::endl; 
            keep_running = false; 
            return; 
        } 
    } else { 
        if (std::fmod(ct, 1.0) < 0.005) {  
            std::cout << "Time: " << ct << " | Mode: " << TEST_MODE  
                      << " | Remaining: " << (current.duration - elapsed) << "s" << std::endl;  
        } 
    } 

    switch (TEST_MODE) 
    { 
    case normal_stand: 
      if (TEST_MODE != last_mode) sport_client.StandUp(); 
      break; 

    case balance_stand:                   
      sport_client.BalanceStand(); 
      break; 

    case velocity_move:  
      sport_client.Move(current_vx, 0, current_vyaw); 
      break; 

    case pitch_control: 
      sport_client.Euler(0, current_pitch, current_yaw); 
      sport_client.BalanceStand(); 
      break; 

    case height_control: 
      sport_client.Euler(0, 0, 0);              
      sport_client.BalanceStand(); 
      break; 

    // --- ESP32 Seeder Commands ---
    case seeder_next:
      sport_client.Euler(0, current_pitch, current_yaw); 
      sport_client.BalanceStand();
      if (!has_sent_message) {
          std::cout << ">>> SENDING 'NEXT' TO ESP32 <<<" << std::endl;
          if (SendMessageToESP32("NEXT")) {
              has_sent_message = true;
          }
      }
      break;

    case seeder_open:
      sport_client.Euler(0, current_pitch, current_yaw); 
      sport_client.BalanceStand();
      if (!has_sent_message) {
          std::cout << ">>> SENDING 'OPEN' TO ESP32 <<<" << std::endl;
          if (SendMessageToESP32("OPEN")) {
              has_sent_message = true;
          }
      }
      break;

    case seeder_close:
      sport_client.Euler(0, current_pitch, current_yaw); 
      sport_client.BalanceStand();
      if (!has_sent_message) {
          std::cout << ">>> SENDING 'CLOSE' TO ESP32 <<<" << std::endl;
          if (SendMessageToESP32("CLOSE")) {
              has_sent_message = true;
          }
      }
      break;

    case stand_down:  
      if (TEST_MODE != last_mode) sport_client.StandDown(); 
      break; 

    case stand_up:  
      if (TEST_MODE != last_mode) sport_client.StandUp(); 
      break; 

    case damp:  
      if (TEST_MODE != last_mode) sport_client.Damp(); 
      break; 

    case recovery_stand:  
      if (TEST_MODE != last_mode) sport_client.RecoveryStand(); 
      break; 

    case sit: 
      if (flag == 0) { sport_client.Sit(); flag = 1; } 
      break; 

    case rise_sit: 
      if (flag == 0) { sport_client.RiseSit(); flag = 1; } 
      break; 

    case stop_move:  
      sport_client.StopMove(); 
      break; 

    case set_walk:
      if (TEST_MODE != last_mode) {
          sport_client.FreeWalk(); 
      }
      break;

    default: 
      sport_client.StopMove(); 
    } 

    last_mode = TEST_MODE; 
  }; 

  void GetInitState() 
  { 
    px0 = state.position()[0]; 
    py0 = state.position()[1]; 
    yaw0 = state.imu_state().rpy()[2]; 

    std::cout << "Initial Position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl; 
  }; 

  void HighStateHandler(const void *message) 
  { 
    state = *(unitree_go::msg::dds_::SportModeState_ *)message; 
  }; 

  unitree_go::msg::dds_::SportModeState_ state; 
  unitree::robot::go2::SportClient sport_client; 
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber; 

  double px0, py0, yaw0;  
  double ct = 0;          
  int flag = 0;           
  float dt = 0.005;       

  float current_vx = 0.0; 
  float current_vyaw = 0.0; 
  float current_pitch = 0.0; 
  float current_yaw = 0.0;
}; 

int main(int argc, char **argv) 
{ 
  if (argc < 2) 
  { 
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl; 
    exit(-1); 
  } 

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]); 
  Custom custom; 

  sleep(1);  
  custom.GetInitState();  

  unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::RobotControl, &custom)); 

  while (custom.keep_running) 
  { 
    sleep(1);
  } 

  std::cout << "Script sequence complete. Shutting down cleanly..." << std::endl;
  return 0;
}
