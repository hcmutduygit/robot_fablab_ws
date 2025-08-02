#!/usr/bin/env python3
# Import packages
import sys
from mqtt_base import MQTTTemplate, get_topic

class NamePublisher(MQTTTemplate):
    def __init__(self):
        super().__init__()  # Uses config from mqtt_base
        self.topic = get_topic("attendance")
    
    def on_connect_callback(self, client, userdata, flags, rc):
        # Publish the attendance data when connected
        client.publish(self.topic, self.message)
        print(f"Published attendance data: {self.message}")
        client.disconnect()
    
    def publish_attendance(self, mqtt_msg, user_name):
        attendance_data = {
            "message": mqtt_msg,
            "user": user_name
        }
        self.message = self.format_message(attendance_data)
        self.publish_and_exit(self.topic, attendance_data)
    
    def format_message(self, data):
        import json
        return json.dumps(data)

if __name__ == "__main__":
    # Get message from command line argument
    if len(sys.argv) > 1:
        mqtt_msg = sys.argv[1]
        user_name = sys.argv[2] if len(sys.argv) > 2 else "Unknown"
    else:
        mqtt_msg = "Default"
        user_name = "Unknown"
    
    print(f"Received arguments: MSG='{mqtt_msg}', USER='{user_name}'")
    
    # Create and use publisher
    publisher = NamePublisher()
    publisher.publish_attendance(mqtt_msg, user_name)
