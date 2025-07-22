#!/usr/bin/env python3
# Import package
import paho.mqtt.client as mqtt
import sys

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

# Define on_connect event Handler
def on_connect(mosq, obj, rc):
    if rc == 0:
        print("Connected to MQTT Broker")
    else:
        print(f"Failed to connect to MQTT Broker, code: {rc}")

# Define on_publish event Handler
def on_publish(client, userdata, mid):
    print(f"Message '{MQTT_MSG}' published for {user_name}")

# Initiate MQTT Client
mqttc = mqtt.Client()

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
