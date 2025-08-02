#!/usr/bin/env python3
"""
MQTT Base Module - All-in-one MQTT solution
Includes configuration and template base class
"""
import paho.mqtt.client as mqtt
import json
import sys
import time

# ======================================
# CONFIGURATION SECTION
# ======================================

# MQTT Broker Configuration
MQTT_CONFIG = {
    "host": "10.189.8.76",
    "port": 1883,
    "keepalive": 5,
    "timeout": 60
}

# Topics Configuration
TOPICS = {
    "battery": "robot/battery",
    "velocity": "robot/velocity", 
    "attendance": "robot/attendance",
    "status": "robot/status",
    "sensors": "robot/sensors"
}

# Message Settings
MESSAGE_CONFIG = {
    "qos": 0,
    "retain": False,
    "delay": 1  # seconds to wait before disconnecting
}

def get_mqtt_config():
    """Get MQTT broker configuration"""
    return MQTT_CONFIG

def get_topic(topic_name):
    """Get topic by name"""
    return TOPICS.get(topic_name, f"robot/{topic_name}")

def get_message_config():
    """Get message configuration"""
    return MESSAGE_CONFIG

# ======================================
# TEMPLATE BASE CLASS SECTION
# ======================================

class MQTTTemplate:
    def __init__(self, host=None, port=None, keepalive=None):
        # Load configuration
        config = get_mqtt_config()
        msg_config = get_message_config()
        
        self.host = host or config["host"]
        self.port = port or config["port"] 
        self.keepalive = keepalive or config["keepalive"]
        self.delay = msg_config["delay"]
        self.qos = msg_config["qos"]
        
        self.client = None
        self._create_client()
    
    def _create_client(self):
        """Create MQTT client with simple API"""
        self.client = mqtt.Client()
        self._setup_callbacks()
        print("Using paho-mqtt simple API")
    
    def _setup_callbacks(self):
        """Setup callbacks for simple API"""
        self.client.on_connect = self._on_connect
        self.client.on_publish = self._on_publish
        self.client.on_subscribe = self._on_subscribe
        self.client.on_message = self._on_message
    
    # Simple Callbacks
    def _on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT Broker with result code {rc}")
        self.on_connect_callback(client, userdata, flags, rc)
    
    def _on_publish(self, client, userdata, mid):
        print(f"Message published with mid: {mid}")
        self.on_publish_callback(client, userdata, mid)
    
    def _on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed to MQTT Topic")
        self.on_subscribe_callback(client, userdata, mid, granted_qos)
    
    def _on_message(self, client, userdata, msg):
        self.on_message_callback(client, userdata, msg)
    
    # Override these methods in subclasses
    def on_connect_callback(self, client, userdata, flags, rc):
        pass
    
    def on_publish_callback(self, client, userdata, mid):
        pass
    
    def on_subscribe_callback(self, client, userdata, mid, granted_qos):
        pass
    
    def on_message_callback(self, client, userdata, msg):
        pass
    
    def connect(self):
        """Connect to MQTT broker"""
        try:
            self.client.connect(self.host, self.port, self.keepalive)
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def publish(self, topic, message, qos=0):
        """Publish message to topic"""
        if isinstance(message, dict):
            message = json.dumps(message)
        return self.client.publish(topic, message, qos)
    
    def subscribe(self, topic, qos=0):
        """Subscribe to topic"""
        return self.client.subscribe(topic, qos)
    
    def loop_forever(self):
        """Keep the network loop running"""
        self.client.loop_forever()
    
    def loop_start(self):
        """Start the network loop in background"""
        self.client.loop_start()
    
    def loop_stop(self):
        """Stop the network loop"""
        self.client.loop_stop()
    
    def disconnect(self):
        """Disconnect from broker"""
        self.client.disconnect()
    
    def publish_and_exit(self, topic, message, delay=None):
        """Publish message and exit (for simple publishers)"""
        delay = delay or self.delay
        if self.connect():
            self.publish(topic, message)
            self.loop_start()
            time.sleep(delay)
            self.loop_stop()
            self.disconnect()
        else:
            print("Failed to connect to MQTT broker")
