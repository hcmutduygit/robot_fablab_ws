#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from utils.msg import waypoints
import json
import paho.mqtt.client as mqtt

# --- Config MQTT ---
MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_TOPIC = "robot/waypoints"

# --- Callback function when received MQTT message ---
def on_message(mosq, obj, msg):
    try:
        rospy.loginfo("=== Received MQTT message from topic: %s ===", msg.topic)
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)

        # List of waypoint [{"x":..,"y":..}, ...]
        if isinstance(data, list):
            for point in data:
                if "x" in point and "y" in point:
                    wp = waypoints()
                    wp.direction_x = point["x"]
                    wp.direction_y = point["y"]
                    pub.publish(wp)
                    rospy.loginfo("=== Published waypoint: x=%.2f, y=%.2f ===", wp.direction_x, wp.direction_y)
        
        # One waypoint {"x":..,"y":..}
        elif isinstance(data, dict) and "x" in data and "y" in data:
            wp = waypoints()
            wp.direction_x = data["x"]
            wp.direction_y = data["y"]
            pub.publish(wp)
            rospy.loginfo("=== Published waypoint: x=%.2f, y=%.2f ===", wp.direction_x, wp.direction_y)

        else:
            rospy.logwarn("Unvalid JSON data: %s", payload)

    except Exception as e:
        rospy.logerr("Error handle message MQTT: %s", e)

# --- Callback when connect MQTT ---
def on_connect(mosq, obj, flags, rc):
    rospy.loginfo("Connect to MQTT broker success (rc=%s)", str(rc))
    mosq.subscribe(MQTT_TOPIC, 0)

# --- Callback when subscribe ---
def on_subscribe(mosq, obj, mid, granted_qos):
    rospy.loginfo("Subscribe topic: %s", MQTT_TOPIC)

# --- Main function ---
if __name__ == '__main__':
    rospy.init_node('mqtt_waypoint_subscriber', anonymous=True)
    
    # Use latch=True to keep last message for late subscribers
    pub = rospy.Publisher('waypoints', waypoints, queue_size=10, latch=True)
    rospy.loginfo("=== Waypoints publisher created (latched) ===")

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

    # --- Runing ROS + MQTT ---
    while not rospy.is_shutdown():
        mqttc.loop(0.1)  # handle MQTT in loop
        rospy.sleep(0.1)
