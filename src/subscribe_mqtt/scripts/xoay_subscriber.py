#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import struct
import serial
import time
import json
import paho.mqtt.client as mqtt

# --- Config MQTT ---
MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_TOPIC = "robot/xoay"

# --- Config CAN ---
CAN_PORT = "/dev/usbcan"
CAN_BAUDRATE = 2000000
CAN_ID = 0x40

# Global variable to track last sent value
last_sent_value = None

# --- Callback function when received MQTT message ---
def on_message(mosq, obj, msg):
    global last_sent_value
    try:
        rospy.loginfo("=== Received MQTT message from topic: %s ===", msg.topic)
        payload = msg.payload.decode("utf-8")
        rospy.loginfo("Raw payload: %s", payload)
        
        # Try to parse as JSON first
        try:
            data = json.loads(payload)
            if isinstance(data, dict) and "angle" in data:
                angle_value = float(data["angle"])
            else:
                # If not JSON with "angle" key, try direct float conversion
                angle_value = float(payload)
        except:
            # If JSON parsing fails, try direct float conversion
            angle_value = float(payload)
        
        rospy.loginfo("Angle value: %.2f degrees", angle_value)
        
        # Only send if value is different from last sent value
        if angle_value != last_sent_value:
            send_angle_to_can(angle_value)
            last_sent_value = angle_value
        else:
            rospy.loginfo("(Skipped: Same value as last sent)")
        
    except ValueError as e:
        rospy.logerr("Error: Could not convert payload to float: %s (Error: %s)", payload, str(e))
    except Exception as e:
        rospy.logerr("Error handle message MQTT: %s", e)

def send_angle_to_can(angle_value):
    """Send angle value to CAN bus via serial port"""
    try:
        # Convert angle to 16-bit integer (scaled by 100)
        angle_scaled = int(angle_value * 100)
        
        # Clamp to 16-bit range (-32768 to 32767)
        if angle_scaled > 32767:
            angle_scaled = 32767
        elif angle_scaled < -32768:
            angle_scaled = -32768
        
        # Convert to bytes (big-endian)
        angle_bytes = struct.pack('>h', angle_scaled)
        
        # Create CAN frame according to WaveshareCAN protocol:
        # [0xAA, CMD, IDL, IDH, DATA(8), 0x55]
        frame = bytearray()
        frame.append(0xAA)  # Start byte
        frame.append(0xC8)  # CMD byte
        frame.append(CAN_ID & 0xFF)  # IDL
        frame.append((CAN_ID >> 8) & 0xFF)  # IDH
        
        # Add angle data (2 bytes) + padding (6 bytes)
        frame.extend(angle_bytes)
        frame.extend([0x00] * 6)  # Padding to 8 bytes
        
        frame.append(0x55)  # End byte
        
        # Debug: Print frame in hex format
        frame_hex = ' '.join('{:02X}'.format(b) for b in frame)
        rospy.loginfo("CAN Frame to send (HEX): %s", frame_hex)
        
        # Print angle data hex only
        angle_hex = ' '.join('{:02X}'.format(b) for b in angle_bytes)
        rospy.loginfo("Angle data (HEX): %s (Value: %.2f degrees)", angle_hex, angle_value)
        
        # Open serial port and send
        ser = serial.Serial(CAN_PORT, CAN_BAUDRATE, timeout=1.0)
        time.sleep(0.1)  # Wait for port to be ready
        
        bytes_written = ser.write(frame)
        ser.close()
        
        if bytes_written == len(frame):
            rospy.loginfo("✓ Successfully sent %d bytes to CAN bus (ID: 0x%02X, Angle: %.2f degrees)", 
                         bytes_written, CAN_ID, angle_value)
            rospy.loginfo("Exit after sending CAN data successfully")
            exit(0)
        else:
            rospy.logwarn("⚠ Warning: Only %d of %d bytes sent to CAN bus", bytes_written, len(frame))
        
    except serial.SerialException as e:
        rospy.logerr("✗ Serial Error: Could not open port %s. Error: %s", CAN_PORT, str(e))
    except Exception as e:
        rospy.logerr("✗ Error sending to CAN bus: %s", str(e))

# --- Callback when connect MQTT ---
def on_connect(mosq, obj, flags, rc):
    rospy.loginfo("Connect to MQTT broker success (rc=%s)", str(rc))
    mosq.subscribe(MQTT_TOPIC, 0)

# --- Callback when subscribe ---
def on_subscribe(mosq, obj, mid, granted_qos):
    rospy.loginfo("Subscribe topic: %s", MQTT_TOPIC)

# --- Main function ---
if __name__ == '__main__':
    rospy.init_node('mqtt_xoay_subscriber', anonymous=True)
    rospy.loginfo("=== Starting Xoay Subscriber ===")
    rospy.loginfo("Listening to topic: %s", MQTT_TOPIC)

    # Create client MQTT
    mqttc = mqtt.Client()
    mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    # Var callback
    mqttc.on_connect = on_connect
    mqttc.on_subscribe = on_subscribe
    mqttc.on_message = on_message

    # Connect to broker
    try:
        mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
        rospy.loginfo("Connecting MQTT broker %s:%d ...", MQTT_HOST, MQTT_PORT)
    except Exception as e:
        rospy.logerr("Can't connect to MQTT broker: %s", e)
        exit(1)

    # --- Running ROS + MQTT ---
    while not rospy.is_shutdown():
        mqttc.loop(0.1)  # handle MQTT in loop
        rospy.sleep(0.1)
