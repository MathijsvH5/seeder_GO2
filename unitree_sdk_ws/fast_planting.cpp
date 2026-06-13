/********************************************************************** Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved. 
 ***********************************************************************/

#include <cmath> 
#include <iostream> 
#include <vector> 
#include <unistd.h> 
#include <queue>  

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
  stop_move = 99 
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
  const double SEED_DISTANCE = 1.5; // 3 meters between two seeds 

  // Sequence control
  bool keep_running = true;  
  int last_mode = -1;        
  std::queue<RobotStep> sequence; 
  double step_start_time = 0.0; 

  Custom() 
  { 
    sport_client.SetTimeout(10.0f); 
    sport_client.Init(); 

    double move_duration = SEED_DISTANCE / MOVE_SPEED;   // Calculate the duration dynamically 

    // Sequence for the robot dog 
    // Format: {Mode, Duration (sec), VX, VYAW, Pitch} 

    // Initial Wakeup & Stand 
    sequence.push({stand_up,      1.0}); 
    sequence.push({balance_stand, 0.5}); 
    sequence.push({set_walk,      0.5});
     
    for (int i = 0; i < 5; ++i) {
        // Walk to seed location
        sequence.push({velocity_move, move_duration, MOVE_SPEED, 0.0, 0.0}); 
        sequence.push({stop_move,     1.0}); 

        // Lean forward to plant seed
        sequence.push({pitch_control, 1.0, 0.0, 0.0, 0.4, -0.6}); 
        sequence.push({pitch_control, 1.0, 0.0, 0.0, 0.0, 0.0}); 
        sequence.push({pitch_control, 1.0, 0.0, 0.0, 0.4, 0.6}); 
        sequence.push({pitch_control, 1.0, 0.0, 0.0, 0.0, 0.0}); 
    }

    // Finish and shut down 
    sequence.push({stand_down,    2.0}); 

    // Initialize subscriber for robot state 
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE)); 
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1); 
  }; 

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
          sport_client.FreeWalk(); // Or ClassicWalk(true)
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