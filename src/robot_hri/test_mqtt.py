import paho.mqtt.client as mqtt

# --- Cấu hình broker (IP của laptop bạn) ---
BROKER_ADDRESS = " 10.128.73.236"  # 👉 sửa lại thành IP của laptop
BROKER_PORT = 1883
TOPIC = "robot/move"

# --- Xử lý khi nhận message ---
def on_message(client, userdata, message):
    print("📥 Received from broker:", message.payload.decode())

# --- Kết nối MQTT ---
client = mqtt.Client()
client.connect(BROKER_ADDRESS, BROKER_PORT)
client.subscribe(TOPIC)
client.on_message = on_message

print("🔄 Waiting for message on topic:", TOPIC)
client.loop_forever()
