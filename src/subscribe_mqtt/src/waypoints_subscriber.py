#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from utils.msg import waypoints
import json
import paho.mqtt.client as mqtt

# --- Cấu hình MQTT ---
MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_TOPIC = "robot/waypoints"

# --- Hàm callback khi nhận tin nhắn MQTT ---
def on_message(mosq, obj, msg):
    try:
        rospy.loginfo("Nhận tin nhắn từ topic: %s", msg.topic)
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)

        # Nếu là danh sách toạ độ [{"x":..,"y":..}, ...]
        if isinstance(data, list):
            for point in data:
                if "x" in point and "y" in point:
                    wp = waypoints()
                    wp.x = point["x"]
                    wp.y = point["y"]
                    pub.publish(wp)
                    rospy.loginfo("Đã publish waypoint: x=%.2f, y=%.2f", wp.x, wp.y)
        
        # Nếu chỉ là 1 toạ độ {"x":..,"y":..}
        elif isinstance(data, dict) and "x" in data and "y" in data:
            wp = waypoints()
            wp.x = data["x"]
            wp.y = data["y"]
            pub.publish(wp)
            rospy.loginfo("Đã publish waypoint: x=%.2f, y=%.2f", wp.x, wp.y)

        else:
            rospy.logwarn("Dữ liệu JSON không hợp lệ: %s", payload)

    except Exception as e:
        rospy.logerr("Lỗi khi xử lý message MQTT: %s", e)

# --- Callback khi kết nối MQTT ---
def on_connect(mosq, obj, flags, rc):
    rospy.loginfo("Kết nối MQTT broker thành công (rc=%s)", str(rc))
    mosq.subscribe(MQTT_TOPIC, 0)

# --- Callback khi subscribe ---
def on_subscribe(mosq, obj, mid, granted_qos):
    rospy.loginfo("Đã subscribe topic: %s", MQTT_TOPIC)

# --- Hàm chính ---
if __name__ == '__main__':
    rospy.init_node('mqtt_waypoint_subscriber', anonymous=True)
    pub = rospy.Publisher('waypoints', waypoints, queue_size=10)

    # Tạo client MQTT
    mqttc = mqtt.Client()
    mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    # Gán callback
    mqttc.on_connect = on_connect
    mqttc.on_subscribe = on_subscribe
    mqttc.on_message = on_message

    # Kết nối tới broker
    try:
        mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
        rospy.loginfo("Đang kết nối tới MQTT broker %s:%d ...", MQTT_HOST, MQTT_PORT)
    except Exception as e:
        rospy.logerr("Không thể kết nối MQTT broker: %s", e)
        exit(1)

    # --- Chạy song song ROS + MQTT ---
    while not rospy.is_shutdown():
        mqttc.loop(0.1)  # xử lý sự kiện MQTT trong vòng lặp
        rospy.sleep(0.1)
