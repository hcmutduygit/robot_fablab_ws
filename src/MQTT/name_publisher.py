#!/usr/bin/env python
# Import packages
import sys
from mqtt_base import MQTTTemplate, get_topic

class NamePublisher(MQTTTemplate):
    def __init__(self):
        # Use default config from mqtt_base
        MQTTTemplate.__init__(self)  # Direct call instead of super()
        self.topic = get_topic("attendance")
        self.message_data = {}
    
    def on_connect_callback(self, client, userdata, flags, rc):
        # Publish the attendance data when connected
        self.publish(self.topic, self.message_data)
        print("Published: {} for user: {} at {}".format(self.message_data['message'], self.message_data['user'], self.message_data['time']))
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
    
    print("Received arguments: MSG='{}', USER='{}', TIMESTAMP='{}'".format(MQTT_MSG, user_name, timestamp))
    
    # Create and use publisher
    publisher = NamePublisher()
    publisher.publish_attendance(MQTT_MSG, user_name, timestamp)
