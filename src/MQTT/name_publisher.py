#!/usr/bin/env python3
# Import package
import paho.mqtt.client as mqtt
import sys
import json

# Define Variables
MQTT_HOST = "192.168.0.130"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_TOPIC = "robot/attendance"

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

# Define callback functions for both old and new versions
def on_connect_old(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.publish(MQTT_TOPIC, json.dumps({"message": MQTT_MSG, "user": user_name, "time": timestamp}))
    print(f"Published: {MQTT_MSG} for user: {user_name} at {timestamp}")
    client.disconnect()

def on_connect_new(client, userdata, flags, reason_code, properties=None):
    print(f"Connected with result code {reason_code}")
    client.publish(MQTT_TOPIC, json.dumps({"message": MQTT_MSG, "user": user_name, "time": timestamp}))
    print(f"Published: {MQTT_MSG} for user: {user_name} at {timestamp}")
    client.disconnect()

def on_publish_old(client, userdata, mid):
    print(f"Message published with mid: {mid}")

def on_publish_new(client, userdata, mid, reason_code=None, properties=None):
    print(f"Message published with mid: {mid}")

# Create MQTT client with version compatibility
try:
    # Try new version API
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_connect = on_connect_new
    mqttc.on_publish = on_publish_new
    print("Using paho-mqtt v2 API")
except AttributeError:
    # Use old version API
    mqttc = mqtt.Client()
    mqttc.on_connect = on_connect_old
    mqttc.on_publish = on_publish_old
    print("Using paho-mqtt v1 API")

try:
    # Connect with MQTT Broker
    mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL) 
    
    # Publish message to MQTT Topic 
    mqttc.publish(MQTT_TOPIC, MQTT_MSG)
    print(f"Published: {MQTT_MSG} for user: {user_name} at {timestamp}")
    
    # Wait for message to be sent
    mqttc.loop_start()
    import time
    time.sleep(1)
    mqttc.loop_stop()
    
except Exception as e:
    print(f"Error: {e}")
finally:
    # Disconnect from MQTT_Broker
    mqttc.disconnect()
