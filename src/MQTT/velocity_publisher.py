#!/usr/bin/env python3
# Import packages
import sys
from mqtt_base import MQTTTemplate, get_topic

class VelocityPublisher(MQTTTemplate):
    def __init__(self):
        super().__init__()  # Uses config from mqtt_base
        self.topic = get_topic("velocity")
    
    def on_connect_callback(self, client, userdata, flags, rc):
        # Publish the velocity data when connected
        client.publish(self.topic, self.message)
        print(f"Published velocity data: {self.message}")
        client.disconnect()
    
    def publish_velocity(self, v_left, v_right):
        velocity_data = {
            "v_left": int(v_left),    # Convert to int for CAN pulse data
            "v_right": int(v_right)   # Convert to int for CAN pulse data
        }
        self.message = self.format_message(velocity_data)
        self.publish_and_exit(self.topic, velocity_data)
    
    def format_message(self, data):
        import json
        return json.dumps(data)

if __name__ == "__main__":
    # Get v_left and v_right from command line arguments
    if len(sys.argv) >= 3:
        try:
            # Try to parse as int first (for CAN pulse data), then float
            v_left = int(sys.argv[1])
            v_right = int(sys.argv[2])
        except ValueError:
            # If int parsing fails, try float
            v_left = float(sys.argv[1])
            v_right = float(sys.argv[2])
    else:
        v_left = 0
        v_right = 0
    
    print(f"Received velocity from CAN (ID 0x017): v_left={v_left}, v_right={v_right}")
    
    # Create and use publisher
    publisher = VelocityPublisher()
    publisher.publish_velocity(v_left, v_right)