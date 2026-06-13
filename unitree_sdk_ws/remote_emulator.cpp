#include <iostream>
#include <unistd.h> 
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_factory.hpp> 
#include <unitree/idl/go2/WirelessController_.hpp>

using namespace unitree::robot;

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        return -1;
    }

    // Initialize the global network factory
    ChannelFactory::Instance()->Init(0, argv[1]);
    ChannelPublisher<unitree_go::msg::dds_::WirelessController_> pub("rt/wireless_controller");

    std::cout << "Emulating controller on " << argv[1] << "..." << std::endl;

    unitree_go::msg::dds_::WirelessController_ msg;

    while (true)
    {
        msg.keys(8); // Button 8 (Posture Mode)
        msg.ly(-0.7);  // Height (Low)
        msg.ry(0.5);   // Pitch (Nose Down)
        msg.lx(0.0);
        msg.rx(0.0);

        pub.Write(msg);

        usleep(20000); 
    }

    return 0;
}