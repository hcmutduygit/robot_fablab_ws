#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Arrival Publisher - Publishes robot arrival status over MQTT
Uses MQTT Base Template - Optimized for fast publishing
"""
import sys
import signal
from mqtt_base import MQTTTemplate, get_topic


# --- Signal handler for clean exit ---
def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


class ArrivedPublisher(MQTTTemplate):
    def __init__(self):
        MQTTTemplate.__init__(self)
        self.topic = get_topic("arrival")

    def publish_arrived(self):
        """Publish 'true' to arrived topic"""
        payload = "true"
        self.publish_and_exit(self.topic, payload, delay=0.05)
        print("Published 'true' to arrival topic successfully!")


def main():
    try:
        pub = ArrivedPublisher()
        pub.publish_arrived()
        return 0
    except Exception as e:
        print("Error: {}".format(e))
        return 1


if __name__ == "__main__":
    sys.exit(main())
