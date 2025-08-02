#!/usr/bin/env python3
# Import packages
import sys
from mqtt_base import MQTTTemplate, get_topic

class MQTTSubscriber(MQTTTemplate):
    def __init__(self, topic_name="status"):
        super().__init__()  # Uses config from mqtt_base
        self.topic = get_topic(topic_name) if topic_name in ["battery", "velocity", "attendance", "status", "sensors"] else topic_name
    
    def on_connect_callback(self, client, userdata, flags, rc):
        # Subscribe to topic when connected
        client.subscribe(self.topic, 0)
        print(f"Subscribed to topic: {self.topic}")
    
    def on_message_callback(self, client, userdata, msg):
        try:
            # Try to decode as JSON first
            import json
            data = json.loads(msg.payload.decode())
            print(f"Received JSON: {data}")
        except:
            # If not JSON, print as string
            print(f"Received message: {msg.payload.decode()}")
    
    def start_listening(self):
        """Start listening for messages"""
        if self.connect():
            print(f"Listening for messages on topic: {self.topic}")
            self.loop_forever()
        else:
            print("Failed to connect to MQTT broker")

if __name__ == "__main__":
    # Get topic from command line argument
    if len(sys.argv) > 1:
        topic_name = sys.argv[1]
    else:
        topic_name = "status"
    
    print(f"Starting subscriber for topic: {topic_name}")
    
    # Create and use subscriber
    subscriber = MQTTSubscriber(topic_name=topic_name)
    try:
        subscriber.start_listening()
    except KeyboardInterrupt:
        print("\nStopping subscriber...")
        subscriber.disconnect()
