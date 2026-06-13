/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <queue> 

#include <fcntl.h>    // Contains file controls like O_RDWR
#include <errno.h>    // Error integer and strerror() function
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()
#include <string>
#include <cstring>    // strerror
#include <cerrno>     // errno

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
  stop_move = 99,
  // seeder integration
  seeder_next = 100,
  seeder_open = 101,
  seeder_close = 102
};

// structure of the actions (mode, duration, vx, vyaw, pitch)
struct RobotStep {
    test_mode mode;
    double duration;
    double vx = 0.0;
    double vyaw = 0.0;
    double pitch = 0.0;
};

class Custom
{
public:
  // constants
  const double MOVE_SPEED = 0.6;    // 0.6 m/s 
  const double SEED_DISTANCE = 2.0; // 3 meters between two seeds

  // sequence control
  bool keep_running = true; 
  int last_mode = -1;       
  std::queue<RobotStep> sequence;
  double step_start_time = 0.0;

  int serial_port = -1; // NEW: Holds the file descriptor for Bluetooth

  Custom()
  {
    // --- NEW: OPEN THE BLUETOOTH PORT ---
    serial_port = open("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port < 0) {
        std::cerr << "WARNING: Could not open /dev/rfcomm0. Is the ESP32 bound?" << std::endl;
    } else {
        std::cout << "Successfully connected to Bluetooth ESP32!" << std::endl;
        
        // Configure port for raw text output
        struct termios tty;
        tcgetattr(serial_port, &tty);
        cfmakeraw(&tty);
        //tty.c_oflag &= ~OPOST; // Raw output
        tcsetattr(serial_port, TCSANOW, &tty);

        tcflush(serial_port, TCIOFLUSH); // Clears the pipes!
    }

    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    double move_duration = SEED_DISTANCE / MOVE_SPEED;   // Calculate the duration dynamically based on distance and speed

    // Sequence for the robot dog
    // Format: {Mode, Duration (sec), VX, VYAW, Pitch}

    // Initial Wakeup & Stand
    sequence.push({stand_up,      1.0});
    sequence.push({balance_stand, 0.5});
    
    // Walk to the location for the seed
    sequence.push({velocity_move, move_duration, MOVE_SPEED, 0.0, 0.0});
    sequence.push({stop_move,     1.0}); // Buffer to stabilize
    
    // Plant seed
    sequence.push({seeder_next,   2.0, 0.0, 0.0, 0.0}); // Load next seed by sending 'next' to ESP32
    sequence.push({pitch_control,  2.0, 0.0, 0.0, 0.75}); // Lean forward
    sequence.push({seeder_open, 1.0, 0.0, 0.0, 0.75}); // Send 'open' to the ESP32 
    sequence.push({pitch_control,  2.0, 0.0, 0.0, 0.0}); // Stand back
    sequence.push({seeder_close,  1.0, 0.0, 0.0, 0.0}); // Send 'close' to the ESP32

    // Walk to the location for the seed
    sequence.push({velocity_move, move_duration, MOVE_SPEED, 0.0, 0.0});
    sequence.push({stop_move,     1.0}); // Buffer to stabilize
    
    // Plant seed
    sequence.push({seeder_next,   2.0, 0.0, 0.0, 0.0}); // Load next seed by sending 'next' to ESP32
    sequence.push({pitch_control,  1.0, 0.0, 0.0, 0.75}); // Lean forward
    sequence.push({seeder_open, 1.0, 0.0, 0.0, 0.75}); // Send 'open' to the ESP32 
    sequence.push({pitch_control,  1.0, 0.0, 0.0, 0.0}); // Stand back
    sequence.push({seeder_close,  1.0, 0.0, 0.0, 0.0}); // Send 'close' to the ESP32

    // Finish and shut down
    sequence.push({stand_down,    1.0});

    // Initialize subscriber for robot state
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
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
      
      // 1. Throw away any pending "OK" messages from the ESP32
      char trash_buffer[256];
      read(serial_port, trash_buffer, sizeof(trash_buffer)); 
      
      // 2. Send the new message
      std::string payload = msg + "\n"; 
      int bytes_written = write(serial_port, payload.c_str(), payload.length());
      
      if (bytes_written < 0) {
          std::cerr << "Failed to write to Bluetooth port." << std::endl;
      }
  }

  void RobotControl()
  {
    ct += dt;

    // Safety check: Exit if the queue is empty
    if (sequence.empty()) {
        keep_running = false;
        return;
    }

    // Get the current active step
    RobotStep& current = sequence.front();

    // Update the local variables based on the step
    int TEST_MODE = current.mode;
    current_vx = current.vx;
    current_vyaw = current.vyaw;
    current_pitch = current.pitch;

    // Check if time for this step has expired
    double elapsed = ct - step_start_time; 
    if (elapsed >= current.duration) { 
        std::cout << "Time: " << ct << "s | Completed mode: " << TEST_MODE << std::endl; 
        sequence.pop();              
        step_start_time = ct;       
         
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

    // Different modes switch case
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
      sport_client.Euler(0, current_pitch, 0);
      sport_client.BalanceStand();
      break;

    case height_control:
      sport_client.Euler(0, 0, 0);             
      sport_client.BalanceStand();
      break;

    case seeder_next:
      sport_client.Euler(0, current_pitch, 0); 
      sport_client.BalanceStand();
      if (TEST_MODE != last_mode) {
          std::cout << ">>> SENDING 'next' TO ESP32 <<<" << std::endl;
          SendMessageToESP32("NEXT"); //<--- I still need to write this function
      }
      break;

    case seeder_open:
      sport_client.Euler(0, current_pitch, 0); 
      sport_client.BalanceStand();
      if (TEST_MODE != last_mode) {
          std::cout << ">>> SENDING 'open' TO ESP32 <<<" << std::endl;
          SendMessageToESP32("OPEN"); //<--- I still need to write this function
      }
      break;

    case seeder_close:
      sport_client.Euler(0, current_pitch, 0); 
      sport_client.BalanceStand();
      if (TEST_MODE != last_mode) {
          std::cout << ">>> SENDING 'close' TO ESP32 <<<" << std::endl;
          SendMessageToESP32("CLOSE"); //<--- I still need to write this function
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


    case stop_move: 
      sport_client.StopMove();
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
    std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
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