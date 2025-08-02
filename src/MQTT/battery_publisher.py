#!/usr/bin/env python3
# Import packages
import sys
from mqtt_base import MQTTTemplate, get_topic

class BatteryPublisher(MQTTTemplate):
    def __init__(self):
        super().__init__()  # Uses config from mqtt_base
        self.topic = get_topic("battery")
    
    def on_connect_callback(self, client, userdata, flags, rc):
        # Publish the battery data when connected
        client.publish(self.topic, self.message)
        print(f"Published battery data: {self.message}")
        client.disconnect()
    
    def publish_battery(self, battery_level):
        self.message = str(battery_level)  # Keep simple string format like original
        self.publish_and_exit(self.topic, str(battery_level))

if __name__ == "__main__":
    # Get battery level from command line argument
    if len(sys.argv) > 1:
        battery_level = sys.argv[1]
    else:
        battery_level = "99"
    
    print(f"Publishing battery level: {battery_level}%")
    
    # Create and use publisher
    publisher = BatteryPublisher()
    publisher.publish_battery(battery_level)
