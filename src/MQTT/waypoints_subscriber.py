#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Waypoints Subscriber - Subscribes waypoints data over MQTT
Uses MQTT Base Template - Optimized for fast publishing
"""
import sys
import signal
from mqtt_base import MQTTTemplate, get_topic

# Signal handler for clean exit
def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

class WaypointsSubscriber(MQTTTemplate):
    def __init__(self):
        MQTTTemplate.__init__(self)
        self.topic = get_topic("")

    def subscribe_waypoints(self):
        