#!/usr/bin/env python3
# Import packages
import sys
from mqtt_base import MQTTTemplate, get_topic

class NamePublisher(MQTTTemplate):
    def __init__(self):
        # Use default config from mqtt_base
        super().__init__()
        self.topic = get_topic("attendance")
        self.message_data = {}
    
    def on_connect_callback(self, client, userdata, flags, rc):
        # Publish the attendance data when connected
        self.publish(self.topic, self.message_data)
        print(f"Published: {self.message_data['message']} for user: {self.message_data['user']} at {self.message_data['time']}")
        client.disconnect()
    
    def publish_attendance(self, mqtt_msg, user_name, timestamp):
        self.message_data = {
            "message": mqtt_msg,
            "user": user_name,
            "time": timestamp
        }
        self.publish_and_exit(self.topic, self.message_data)

if __name__ == "__main__":
    # Get message from command line argument
    if len(sys.argv) > 1:
        MQTT_MSG = sys.argv[1]
        user_name = sys.argv[2] 
        timestamp = sys.argv[3] if len(sys.argv) > 3 else "Unknown"
    else:
        MQTT_MSG = "Default"
        user_name = "Unknown"
        timestamp = "Unknown"
    
    print(f"Received arguments: MSG='{MQTT_MSG}', USER='{user_name}', TIMESTAMP='{timestamp}'")
    
    # Create and use publisher
    publisher = NamePublisher()
    publisher.publish_attendance(MQTT_MSG, user_name, timestamp)
