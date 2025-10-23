

import paho.mqtt.client as mqtt

# --- Cấu hình MQTT ---
MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_TOPIC = "robot/waypoints"

# --- Callback khi kết nối ---
def on_connect(mosq, obj, flags, rc):
    print("Kết nối thành công tới broker (rc=%s)" % str(rc))
    mosq.subscribe(MQTT_TOPIC, 0)

# --- Callback khi subscribe thành công ---
def on_subscribe(mosq, obj, mid, granted_qos):
    print("Đã subscribe topic: %s" % MQTT_TOPIC)

# --- Callback khi nhận được tin nhắn ---
def on_message(mosq, obj, msg):
    try:
        print("Nhận tin nhắn từ topic: %s" % msg.topic)
        print("Nội dung: %s" % msg.payload.decode("utf-8"))
    except Exception as e:
        print("Lỗi khi xử lý message:", e)

# --- Tạo client MQTT ---
mqttc = mqtt.Client()

# --- Thiết lập username/password ---
mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

# --- Gán callback ---
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_message = on_message

# --- Kết nối tới broker ---
try:
    mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
    print("Đang kết nối tới MQTT broker %s:%d ..." % (MQTT_HOST, MQTT_PORT))
except Exception as e:
    print("Không thể kết nối MQTT broker:", e)
    exit(1)

# --- Vòng lặp chính ---
mqttc.loop_forever()
