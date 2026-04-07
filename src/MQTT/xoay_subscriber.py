#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Import packages
import sys
import os
import struct
from mqtt_base import MQTTTemplate, get_topic

class XoaySubscriber(MQTTTemplate):
    def __init__(self):
        MQTTTemplate.__init__(self)  # Direct call instead of super()
        self.topic = "robot/xoay"  # Subscribe to robot/xoay topic
        self.message_count = 0
        self.can_port = "/dev/usbcan"  # CAN port
        self.can_baudrate = 2000000
        self.can_id = 0x40  # CAN ID for sending angle data
    
    def on_connect_callback(self, client, userdata, flags, rc):
        """Subscribe to topic when connected"""
        if rc == 0:
            print("Connected to MQTT broker")
            self.subscribe(self.topic, qos=0)
        else:
            print("Failed to connect, return code {}".format(rc))
    
    def on_subscribe_callback(self, client, userdata, mid, granted_qos):
        """Callback when subscription is acknowledged"""
        print("Subscribed to topic: {}".format(self.topic))
    
    def on_message_callback(self, client, userdata, msg):
        """Handle incoming message from subscribed topic"""
        self.message_count += 1
        # Decode message payload
        try:
            payload = msg.payload.decode('utf-8')
        except:
            payload = str(msg.payload)
        
        # Print the message once
        print("Message received from {}: {}".format(msg.topic, payload))
        
        # Send angle data to CAN bus
        try:
            angle_value = float(payload)
            self.send_angle_to_can(angle_value)
        except ValueError:
            print("Error: Could not convert payload to float: {}".format(payload))
    
    def send_angle_to_can(self, angle_value):
        """Send angle value to CAN bus via serial port"""
        try:
            import serial
            import time
            
            # Convert angle to 16-bit integer (scaled by 100)
            # Example: 45.5 degrees -> 4550
            angle_scaled = int(angle_value * 100)
            
            # Clamp to 16-bit range (-32768 to 32767)
            if angle_scaled > 32767:
                angle_scaled = 32767
            elif angle_scaled < -32768:
                angle_scaled = -32768
            
            # Convert to bytes (big-endian)
            angle_bytes = struct.pack('>h', angle_scaled)  # Big-endian signed short
            
            # Create CAN frame according to WaveshareCAN protocol:
            # [0xAA, CMD, IDL, IDH, DATA(8), 0x55]
            frame = bytearray()
            frame.append(0xAA)  # Start byte
            frame.append(0xC8)  # CMD byte
            frame.append(self.can_id & 0xFF)  # IDL
            frame.append((self.can_id >> 8) & 0xFF)  # IDH
            
            # Add angle data (2 bytes) + padding (6 bytes)
            frame.extend(angle_bytes)
            frame.extend([0x00] * 6)  # Padding to 8 bytes
            
            frame.append(0x55)  # End byte
            
            # Open serial port and send
            ser = serial.Serial(self.can_port, self.can_baudrate, timeout=1.0)
            time.sleep(0.1)  # Wait for port to be ready
            ser.write(frame)
            ser.close()
            
            print("Sent angle {} degrees to CAN bus (ID: 0x{:02X})".format(angle_value, self.can_id))
            
        except ImportError:
            print("Error: pyserial not installed. Install with: pip install pyserial")
        except Exception as e:
            print("Error sending to CAN bus: {}".format(str(e)))

if __name__ == "__main__":
    print("Starting Xoay Subscriber...")
    print("Listening to topic: robot/xoay")
    
    # Create subscriber
    subscriber = XoaySubscriber()
    
    # Connect and start listening
    if subscriber.connect():
        print("Connected successfully, waiting for messages...")
        try:
            # Keep listening for messages
            subscriber.loop_forever()
        except KeyboardInterrupt:
            print("\nDisconnecting...")
            subscriber.disconnect()
    else:
        print("Failed to connect to MQTT broker")
        sys.exit(1)
