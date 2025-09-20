#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Location Publisher - Publishes robot location data over MQTT
Uses MQTT Base Template - Optimized for fast publishing
"""
import sys
import signal
from mqtt_base import MQTTTemplate, get_topic

# Signal handler for clean exit
def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

class LocationPublisherOnce(MQTTTemplate):
    def __init__(self):
        # Initialize with parent class
        MQTTTemplate.__init__(self)
        self.topic = get_topic("location")
    
    # def on_connect_callback(self, client, userdata, flags, rc):
        # print("LocationPublisher connected with rc={}".format(rc))
    
    # def on_publish_callback(self, client, userdata, mid):
        # print("Location data published with id: {}".format(mid))
    
    def publish_location(self, x, y, theta):
        """Publish robot location data (x, y, theta) and exit immediately
        
        Args:
            x (float): X coordinate in meters
            y (float): Y coordinate in meters  
            theta (float): Orientation in degrees
        """
        location_data = {
            "x": x,
            "y": y,
            "theta": theta
        }
        
        # Use publish_and_exit for quick publishing
        self.publish_and_exit(self.topic, location_data, delay=0.01)  # Giảm delay xuống 10ms
        # print("Published location: x={:.2f}, y={:.2f}, theta={:.2f}".format(x, y, theta))


def main():
    if len(sys.argv) < 4:
        print("Usage: python2 location_publisher.py <x> <y> <theta>")
        return 1
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3])
        
        publisher = LocationPublisherOnce()
        publisher.publish_location(x, y, theta)
        return 0
    except Exception as e:
        print("Error: {}".format(e))
        return 1

if __name__ == "__main__":
    sys.exit(main())

