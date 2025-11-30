#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Telemetry Publisher - Publishes robot telemetry data (imu, odom, send)
Usage: python2 telemetry_publisher.py <imu> <odom> <send>
"""
from mqtt_base import MQTTTemplate, get_topic
import json
import time
import sys

class TelemetryPublisher(MQTTTemplate):
    def __init__(self, host=None, port=None):
        super(TelemetryPublisher, self).__init__(host, port)
        self.topic = get_topic("telemetry")
    
    def publish_telemetry(self, imu=0, odom=0, send=0):
        """
        Publish telemetry data
        
        Args:
            imu (int): IMU value
            odom (int): Odometry value
            send (int): Send value
        """
        message = {
            "imu": int(imu),
            "odom": int(odom),
            "send": int(send)
        }
        
        result = self.publish(self.topic, message, qos=self.qos)
        return result
    
    def publish_telemetry_once(self, imu=0, odom=0, send=0):
        """
        Publish telemetry data once and disconnect
        
        Args:
            imu (int): IMU value
            odom (int): Odometry value
            send (int): Send value
        """
        message = {
            "imu": int(imu),
            "odom": int(odom),
            "send": int(send)
        }
        
        self.publish_and_exit(self.topic, message)


def main():
    """
    Main function - handles both command line usage and example loop
    Command line: python2 telemetry_publisher.py <imu> <odom> <send>
    No arguments: runs example loop
    """
    # Check if command line arguments provided
    if len(sys.argv) == 4:
        # Command line mode: publish once and exit
        try:
            imu = int(sys.argv[1])
            odom = int(sys.argv[2])
            send = int(sys.argv[3])
            
            publisher = TelemetryPublisher()
            publisher.publish_telemetry_once(imu, odom, send)
            # print("Published telemetry: imu={}, odom={}, send={}".format(imu, odom, send))
            
        except ValueError:
            print("Error: All arguments must be integers")
            print("Usage: python2 telemetry_publisher.py <imu> <odom> <send>")
            sys.exit(1)
        except Exception as e:
            print("Error: {}".format(e))
            sys.exit(1)
    
    elif len(sys.argv) == 1:
        # Example mode: continuous publishing loop
        publisher = TelemetryPublisher()
        
        if publisher.connect():
            print("Connected to MQTT broker")
            publisher.loop_start()
            
            try:
                # Example: Publish telemetry data in a loop
                counter = 0
                while counter < 10:
                    imu_value = counter * 10
                    odom_value = counter * 20
                    send_value = counter * 30
                    
                    publisher.publish_telemetry(imu_value, odom_value, send_value)
                    print("Published telemetry: imu={}, odom={}, send={}".format(
                        imu_value, odom_value, send_value))
                    
                    time.sleep(1)
                    counter += 1
                    
            except KeyboardInterrupt:
                print("\nStopping publisher...")
            finally:
                publisher.loop_stop()
                publisher.disconnect()
                print("Disconnected from MQTT broker")
        else:
            print("Failed to connect to MQTT broker")
    
    else:
        print("Usage: python2 telemetry_publisher.py <imu> <odom> <send>")
        print("   or: python2 telemetry_publisher.py  (for example loop)")
        sys.exit(1)


if __name__ == "__main__":
    main()
