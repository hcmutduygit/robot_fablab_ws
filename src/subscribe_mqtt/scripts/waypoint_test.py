

import paho.mqtt.client as mqtt


MQTT_HOST = "45.117.177.157"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 5
MQTT_USERNAME = "client"
MQTT_PASSWORD = "viam1234"
MQTT_TOPIC = "robot/waypoints"


def on_connect(mosq, obj, flags, rc):
    print("Connect successful (rc=%s)" % str(rc))
    mosq.subscribe(MQTT_TOPIC, 0)

# --- Callback when subcriber success ---
def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed topic: %s" % MQTT_TOPIC)

# --- Callback when recieve message ---
def on_message(mosq, obj, msg):
    try:
        print("Recieve message from topic: %s" % msg.topic)
        print("Message: %s" % msg.payload.decode("utf-8"))
    except Exception as e:
        print("Error message:", e)

# --- Create client MQTT ---
mqttc = mqtt.Client()

# --- Manage username/password ---
mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

# --- Var callback ---
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_message = on_message

# --- connect to broker ---
try:
    mqttc.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_INTERVAL)
    print("Connecting to MQTT broker %s:%d ..." % (MQTT_HOST, MQTT_PORT))
except Exception as e:
    print("Can't connect to MQTT broker:", e)
    exit(1)

# --- Main loop ---
mqttc.loop_forever()
