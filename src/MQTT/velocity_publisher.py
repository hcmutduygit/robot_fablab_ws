#!/usr/bin/env python3
# Import package
import paho.mqtt.client as mqtt
import sys
import json

# Define Variables
MQTT_HOST = "10.189.8.76"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_TOPIC = "robot/attendance"