#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys
import signal
from mqtt_base import MQTTTemplate, get_topic

# Signal handler for clean exit
def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

class VelocityPublisherOnce(MQTTTemplate):
    def __init__(self):
        MQTTTemplate.__init__(self)
        self.topic = get_topic("velocity")

    def publish_velocity(self, left, right):
        msg = {"left": left, "right": right}
        self.publish_and_exit(self.topic, msg, delay=0.01)  # Giảm delay xuống 10ms


def main():
    if len(sys.argv) < 3:
        # print("Usage: python2 velocity_publish_once.py <left> <right>")
        return 1
    try:
        # Accept ints or floats
        left = float(sys.argv[1])
        right = float(sys.argv[2])
    except Exception as e:
        print("Invalid arguments: {}".format(e))
        return 1

    pub = VelocityPublisherOnce()
    pub.publish_velocity(left, right)
    # print("Published velocity: left={}, right={}".format(left, right))
    return 0

if __name__ == "__main__":
    sys.exit(main())