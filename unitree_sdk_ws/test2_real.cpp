/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>
#include <iostream>
#include <unistd.h> 
#include <cstdlib>

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
  stop_move = 99
};

int TEST_MODE = normal_stand;

class Custom
{
public:
  bool keep_running = true; // NEW: Flag to safely exit the script
  int last_mode = -1;       // NEW: Tracker to prevent 200Hz command spamming

  Custom()
  {
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
  };

  void RobotControl()
  {
    ct += dt;

    if (std::fmod(ct, 1.0) < 0.005) { 
      std::cout << "Time: " << ct << " | Current Mode: " << TEST_MODE << std::endl; 
    }

  
    // 2. The Timer Sequence
    if (ct < 2.0) {
        TEST_MODE = stand_up; // Spend the first 2 seconds just standing up
    } 
    else if (ct < 4.0) {
        // 3. Bend forward (Smoothly over 3 seconds)
        TEST_MODE = pitch_control;
        current_pitch = 0.75;
        current_yaw = 0.6;
    } 
   /* else if (ct < 4.0) {
        // 3. Bend forward (Smoothly over 3 seconds)
        TEST_MODE = height_control;
        current_height = 0.5;
    } */


    double px_local, py_local, yaw_local;
    double vx_local, vy_local, vyaw_local;
    double px_err, py_err, yaw_err;
    double time_seg, time_temp;

    unitree::robot::go2::PathPoint path_point_tmp;
    std::vector<unitree::robot::go2::PathPoint> path;

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
     // sport_client.BodyHeight(current_height); 
      sport_client.BalanceStand();
      break;

    case stand_down: 
      // Only fire the discrete drop command ONCE
      if (TEST_MODE != last_mode) sport_client.StandDown();
      break;

    case stand_up: 
      // Only fire the discrete rise command ONCE
      if (TEST_MODE != last_mode) sport_client.StandUp();
      break;

    case damp: 
      if (TEST_MODE != last_mode) sport_client.Damp();
      break;

    case recovery_stand: 
      if (TEST_MODE != last_mode) sport_client.RecoveryStand();
      break;

    case sit:
      if (flag == 0)
      {
        sport_client.Sit();
        flag = 1;
      }
      break;

    case rise_sit:
      if (flag == 0)
      {
        sport_client.RiseSit();
        flag = 1;
      }
      break;

    case stop_move: 
      sport_client.StopMove();
      break;

    default:
      sport_client.StopMove();
    }

    // Update last_mode at the end of the loop
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

  // Added variables to control speed dynamically
  float current_vx = 0.0;
  float current_vyaw = 0.0;
  float current_pitch = 0.0;
  float current_height = 0.0;
  float current_yaw = 0.0;
};

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

 
  // 1. Build a custom XML string that forces Unicast directly to the dog's IP
  std::string interface_name = argv[1];
  std::string custom_xml = 
      "<CycloneDDS><Domain><General><Interfaces>"
      "<NetworkInterface name=\"" + interface_name + "\"/>"
      "</Interfaces></General><Discovery><Peers>"
      "<Peer address=\"192.168.123.161\"/>"
      "</Peers></Discovery></Domain></CycloneDDS>";

  // 2. Inject it directly into the program's memory
  setenv("CYCLONEDDS_URI", custom_xml.c_str(), 1);

  // 3. Pass an EMPTY string to Init() so the SDK doesn't overwrite our custom XML!
  unitree::robot::ChannelFactory::Instance()->Init(0, "");

  Custom custom;

  sleep(4); 

  custom.GetInitState(); 
  
  unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::RobotControl, &custom));

  while (custom.keep_running)
  {
    sleep(1); 
  }

  std::cout << "Script sequence complete. Shutting down cleanly..." << std::endl;
  return 0; 
}