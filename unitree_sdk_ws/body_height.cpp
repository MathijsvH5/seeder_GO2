#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <iomanip> // For hex formatting

#include <unitree/robot/channel/channel_subscriber.hpp>
// NEW: Wireless Controller IDL
#include <unitree/idl/go2/WirelessController_.hpp>

using namespace unitree::common;

// NEW: The callback for raw joystick data
void WirelessHandler(const void *message)
{
    // Cast to the correct Go2 SDK2 Type
    const auto* msg = static_cast<const unitree_go::msg::dds_::WirelessController_*>(message);
    
    // On Go2, we use these specific member variables:
    // lx, ly: Left stick X/Y
    // rx, ry: Right stick X/Y
    // keys:   Button bitmask (16-bit)
    
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "[REMOTE] "
              << "Left Stick: (" << msg->lx() << ", " << msg->ly() << ") | "
              << "Right Stick: (" << msg->rx() << ", " << msg->ry() << ") | "
              << "Keys: " << std::hex << msg->keys() << std::dec 
              << std::endl;
}

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  // --- CYCLONE DDS CONFIG ---
  std::string interface_name = argv[1];
  std::string custom_xml = 
      "<CycloneDDS><Domain><General><Interfaces>"
      "<NetworkInterface name=\"" + interface_name + "\"/>"
      "</Interfaces></General><Discovery><Peers>"
      "<Peer address=\"192.168.12.1\"/>"
      "</Peers></Discovery></Domain></CycloneDDS>";

  setenv("CYCLONEDDS_URI", custom_xml.c_str(), 1);
  unitree::robot::ChannelFactory::Instance()->Init(0, "");

  std::cout << "Waiting 4 seconds for DDS Handshake..." << std::endl;
  sleep(4);

  // --- THE WIRETAP ---
  // Topic changed to "rt/wirelesscontroller"
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> remote_suber;
  remote_suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>("rt/wirelesscontroller"));
  remote_suber->InitChannel(WirelessHandler, 10);

  std::cout << "=======================================" << std::endl;
  std::cout << "WIRELESS TAP ACTIVE." << std::endl;
  std::cout << "1. Pick up the remote." << std::endl;
  std::cout << "2. Lower the body height using the joystick combo." << std::endl;
  std::cout << "3. Look for which bytes change!" << std::endl;
  std::cout << "=======================================" << std::endl;

  while (true) {
    sleep(1); 
  }

  return 0; 
}