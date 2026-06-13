#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class EspBridge(Node):
    def __init__(self):
        super().__init__('esp_bridge_node')
        
        self.port_name = '/dev/rfcomm0'
        self.baud_rate = 9600
        
        try:
            self.bt_serial = serial.Serial(self.port_name, self.baud_rate, timeout=1)
            self.get_logger().info(f'Successfully connected to ESP32 on {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to {self.port_name}')
            self.get_logger().error(str(e))
            self.bt_serial = None

        self.subscription = self.create_subscription(
            String,
            'esp_trigger',
            self.listener_callback,
            10)
            
        self.get_logger().info('Listening for commands from CaseController...')

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received ROS trigger: "{command}"')
        
        if self.bt_serial is not None and self.bt_serial.is_open:
            try:
                payload = command + "\n"
                
                self.bt_serial.write(payload.encode('utf-8')) 
                
                self.get_logger().info(f'Sent "{command}" to ESP32')
            except Exception as e:
                self.get_logger().error(f'Lost connection while sending: {e}')
        else:
            self.get_logger().warn(f'Ignored "{command}" - Bluetooth not connected!')

def main(args=None):
    rclpy.init(args=args)
    bridge_node = EspBridge()
    
    try:
        rclpy.spin(bridge_node) 
    except KeyboardInterrupt:
        bridge_node.get_logger().info('Shutting down Bridge...')
    finally:
        if bridge_node.bt_serial and bridge_node.bt_serial.is_open:
            bridge_node.bt_serial.close()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()