# MQTT Base System - All-in-One Solution

## Single File Configuration & Template

All MQTT functionality is now in one file: `mqtt_base.py`
- ✅ Configuration (host, port, topics)
- ✅ Template base class  
- ✅ Version compatibility (paho-mqtt v1 & v2)

## Configuration

Change MQTT settings in `mqtt_base.py`:

```python
# MQTT Broker Configuration
MQTT_CONFIG = {
    "host": "192.168.0.130",    # ← Change broker IP here
    "port": 1883,               # ← Change port here  
    "keepalive": 5,
    "timeout": 60
}

# Topics Configuration  
TOPICS = {
    "battery": "robot/battery",      # ← Change topic names here
    "velocity": "robot/velocity", 
    "attendance": "robot/attendance",
    "status": "robot/status",
    "sensors": "robot/sensors"
}
```

## Usage Examples

### Publishers (auto-use config from mqtt_base.py):
```bash
# Velocity publisher
python3 velocity_publisher.py 5.15 -6.15

# Battery publisher  
python3 battery_publisher.py 85

# Attendance publisher
python3 name_publisher.py "Ky" "MINH KY"
```

### Subscriber (auto-use config from mqtt_base.py):
```bash
# Subscribe to velocity topic
python3 subscriber.py velocity

# Subscribe to battery topic
python3 subscriber.py battery
```

## Creating New Publisher

```python
from mqtt_base import MQTTTemplate, get_topic

class MyPublisher(MQTTTemplate):
    def __init__(self):
        super().__init__()  # Auto-loads config
        self.topic = get_topic("my_topic")
    
    def on_connect_callback(self, client, userdata, flags, rc_or_reason, properties=None):
        client.publish(self.topic, self.message)
        client.disconnect()
```

## Benefits

✅ **Single file** - Everything in `mqtt_base.py`
✅ **One config point** - Change host/port/topics once
✅ **Version safe** - Works with old & new paho-mqtt
✅ **No duplicates** - All publishers inherit same behavior 
✅ **Easy maintenance** - Update base class affects all
