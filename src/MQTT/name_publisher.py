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
    user_name = sys.argv[2] if len(sys.argv) > 2 else "Unknown"
else:
    MQTT_MSG = "Default"
    user_name = "Unknown"

print(f"Received arguments: MSG='{MQTT_MSG}', USER='{user_name}'")

# Define on_connect event Handler
def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"Connected with result code {reason_code}")
    # Publish message after connecting
    client.publish(MQTT_TOPIC, json.dumps({"message": MQTT_MSG, "user": user_name}))
    print(f"Published: {MQTT_MSG} for user: {user_name}")
    client.disconnect()

# Define on_publish event Handler
def on_publish(client, userdata, mid, reason_code=None, properties=None):
    print(f"Message published with mid: {mid}")

# Initiate MQTT Client (fix deprecation warning)
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# Register Event Handlers
mqttc.on_publish = on_publish
mqttc.on_connect = on_connect

try:
    # Connect with MQTT Broker
    mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL) 
    
    # Publish message to MQTT Topic 
    mqttc.publish(MQTT_TOPIC, MQTT_MSG)
    print(f"Published: {MQTT_MSG} for user: {user_name}")
    
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
