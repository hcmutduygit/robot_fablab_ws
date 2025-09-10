#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Location Publisher with ROS Integration - Publishes robot location data over MQTT
Uses MQTT Base Template and ROS1
"""
import sys
import rospy
from nav_msgs.msg import Odometry
import math
from mqtt_base import MQTTTemplate, get_topic

class LocationPublisherROS(MQTTTemplate):
    def __init__(self):
        # Initialize with parent class
        MQTTTemplate.__init__(self)
        self.topic = get_topic("location")
        
        # Initialize ROS node
        rospy.init_node('location_publisher_ros', anonymous=True)
        
        # Subscribe to odometry topic
        self.subscription = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Connect to MQTT broker
        if not self.connect():
            rospy.logerr("Failed to connect to MQTT broker")
            return
        
        self.loop_start()
        rospy.loginfo("Location Publisher started - listening to /odom")
    
    def on_connect_callback(self, client, userdata, flags, rc):
        print("LocationPublisherROS connected with rc={}".format(rc))
    
    def on_publish_callback(self, client, userdata, mid):
        print("Location data published with id: {}".format(mid))

    
    def odom_callback(self, msg):
        """ROS callback function for odometry data"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # Convert quaternion to yaw angle (theta)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        payload = {
            "x": round(x, 3),
            "y": round(y, 3),
            "theta": round(theta, 2)
        }

        try:
            self.publish(self.topic, payload)
            print("[ROS->MQTT] Published: x={:.3f}, y={:.3f}, theta={:.2f}".format(x, y, theta))
        except Exception as e:
            rospy.logerr("Failed to publish to MQTT: {}".format(e))

def main():
    try:
        publisher = LocationPublisherROS()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        print("Shutting down...")
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Clean up MQTT connection
        try:
            publisher.loop_stop()
            publisher.disconnect()
        except:
            pass

if __name__ == '__main__':
    main()
