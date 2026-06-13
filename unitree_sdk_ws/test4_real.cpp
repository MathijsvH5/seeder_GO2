/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <queue> 
#include <fcntl.h>    
#include <errno.h>    
#include <termios.h>  

#include <unistd.h>   // write(), read(), close()
#include <string>
#include <cstring>    // strerror
#include <cerrno>     // errno

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

enum test_mode {
  stand_down = 5,
  stand_up = 6,
  balance_stand = 1,
  pitch_control = 4,
  seeder_open = 101,
  seeder_close = 102,
  seeder_next = 103
};

struct RobotStep { 
    test_mode mode; 
    double duration; 
    double vx = 0.0; 
    double vyaw = 0.0; 
    double pitch = 0.0; 
}; 

class Custom {
public:
  bool keep_running = true; 
  int last_mode = -1;
  bool has_sent_message = false; 
  std::queue<RobotStep> sequence;
  double step_start_time = 0.0;
  int serial_port = -1;

  Custom() {
    serial_port = open("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port >= 0) {
        struct termios tty;
        tcgetattr(serial_port, &tty);
        cfmakeraw(&tty);
        tcsetattr(serial_port, TCSANOW, &tty);

        tcflush(serial_port, TCIOFLUSH); // Clears the pipes!
    }

    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    // --- YOUR SPECIFIC SEQUENCE ---
    sequence.push({stand_down,    2.5}); // 1. Ensure it's down
    sequence.push({stand_up,      3.0}); // 3. Stand up (Allow time to finish move)
    sequence.push({pitch_control, 3.0, 0.0, 0.0, 0.4}); 
    sequence.push({seeder_open,   3.0}); // 2. Send 'OPEN' (while down)
    sequence.push({pitch_control, 3.0, 0.0, 0.0, 0.0}); 
    sequence.push({seeder_close,  3.0}); // 4. Send 'CLOSE' (while standing)
    sequence.push({stand_down,    1}); // 5. Lay back down
    sequence.push({seeder_next,    2.0}); // 6. Send 'NEXT'


    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
  }

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

  void RobotControl() {
    ct += dt;
    if (sequence.empty()) { keep_running = false; return; } 

    RobotStep& current = sequence.front();
    
    // 1. Safely extract the current mode
    int TEST_MODE = current.mode; 
    current_pitch = current.pitch;
    
    // 2. Timer logic
    double elapsed = ct - step_start_time; 
    if (elapsed >= current.duration) { 
        sequence.pop();              
        step_start_time = ct;       
        has_sent_message = false; 
        if (sequence.empty()) keep_running = false;
        return; 
    } 

    // 3. The Switch Statement with the Anti-Spam (last_mode) fix
    switch (TEST_MODE) {
      case stand_down: 
        // ONLY send this command once when the step begins!
        if (TEST_MODE != last_mode) sport_client.StandDown();
        break;

      case stand_up: 
        // ONLY send this command once when the step begins!
        if (TEST_MODE != last_mode) sport_client.StandUp();
        break;

      case pitch_control: 
        // Continuous commands (Euler/Move) MUST be sent at 200Hz, so no 'last_mode' check here
        sport_client.Euler(0, current.pitch, 0); 
        sport_client.BalanceStand(); 
        break; 

      case seeder_open:
        if (!has_sent_message) {
            std::cout << ">>> SENDING 'OPEN' <<<" << std::endl;
            SendMessageToESP32("OPEN");
            has_sent_message = true; 
        }
        break;

      case seeder_close:
        if (!has_sent_message) {
            std::cout << ">>> SENDING 'CLOSE' <<<" << std::endl;
            SendMessageToESP32("CLOSE");
            has_sent_message = true; 
        }
        break;
      
      case seeder_next:
        if (!has_sent_message) {
            std::cout << ">>> SENDING 'NEXT' <<<" << std::endl;
            SendMessageToESP32("NEXT");
            has_sent_message = true; 
        }
        break;
    }

    // 4. Update last_mode so we don't spam transition commands on the next loop!
    last_mode = TEST_MODE;
  }


  void HighStateHandler(const void *message) {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
  }

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
  double ct = 0;         
  float dt = 0.005;    
  float current_pitch = 0.0;   
};

int main(int argc, char **argv) {
  if (argc < 2) return -1;
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  Custom custom;
  sleep(1); 
  unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::RobotControl, &custom));
  while (custom.keep_running) sleep(1);
  return 0; 
}