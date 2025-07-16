#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tft
import matplotlib.pyplot as plt
import numpy as np
import math
import time

class AdaptiveLOSTrajectoryController:
    def __init__(self):
        rospy.init_node('adaptive_los_trajectory_controller')

        self.ROBOT_NAME = "robot_fablab"
        self.cmd_vel_pub = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_heading_error = 0.0
        self.last_time = None
        self.delta_min = 0.5
        self.delta_max = 0.8

        self.KP = 2.5
        self.LINEAR_SPEED = 1.5
        self.ANGULAR_SPEED = 2
        self.GOAL_RADIUS = 1.5
        self.MAX_LINEAR_SPEED = 2
        self.MAX_ANGULAR_SPEED = 2.5
        self.filtered_linear_x = 0.0
        self.filtered_angular_z = 0.0

        # self.waypoints = [(0,0),(7.765,-4.42),(19.68,-12.1),(28.912,3.44),(35.22, 12.78),(39.6, 10.324),(50.53,2.95)]
        self.waypoints = [(0,0),(8,0),(8,6.5),(25,6.5),(25,24),(30,24)]
        # self.waypoints = [(0,0),(4,0)]
        self.x_data = []
        self.y_data = []
        self.angular_data = []
        self.time_data = []
        self.start_time = time.time()
        self.target_heading_data = []
        self.linear_velocity_data = []
        self.angular_velocity_data = []
        self.current_waypoint_index = 0
    def model_states_callback(self, msg):
        try:
            index = msg.name.index(self.ROBOT_NAME)
            position = msg.pose[index].position
            orientation = msg.pose[index].orientation
            _, _, self.current_theta = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
            self.current_x = position.x
            self.current_y = position.y
            self.x_data.append(position.x)
            self.y_data.append(position.y)
        except ValueError:
            pass

    def low_pass_filter(self, prev_value, new_value, alpha=0.2):
        return alpha * new_value + (1 - alpha) * prev_value

    def euler_from_quaternion(self, x, y, z, w):
        return tft.euler_from_quaternion([x, y, z, w])

    def get_heading(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def control_adaptive_los(self, goal_x, goal_y,previous_x,previous_y):        
        alpha_k = self.get_heading(previous_x,previous_y,goal_x,goal_y)
        s_k_1 = ( goal_x - previous_x) * math.cos(alpha_k) + ( goal_y- previous_y) * math.sin(alpha_k)

        cross_track = -(self.current_x - previous_x) * math.sin(alpha_k) + (self.current_y - previous_y) * math.cos(alpha_k)
        long_track = (self.current_x - previous_x) * math.cos(alpha_k) + (self.current_y - previous_y) * math.sin(alpha_k)

        delta = (self.delta_max - self.delta_min) *  math.exp(-0.7 * (cross_track**2)) + self.delta_min
        
        twist = Twist()
        target_heading = self.normalize_angle(alpha_k + math.atan(-cross_track/delta))
        self.target_heading_data.append(target_heading)

        heading_error = self.normalize_angle(target_heading - self.current_theta)
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 0.01
        derivative = (heading_error - self.last_heading_error) / dt if dt > 0 else 0.0

        angular_z = (self.ANGULAR_SPEED * heading_error + self.KP * derivative)
        angular_z = max(min(angular_z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        self.filtered_angular_z = angular_z

        dist_to_goal = abs(s_k_1 - long_track)
        per_dist = abs(s_k_1 - long_track)/s_k_1
        if abs(heading_error) > 0.1:
            twist.linear.x = max(min(self.MAX_LINEAR_SPEED/2 , self.LINEAR_SPEED), 0.05)
        else:
            twist.linear.x = max(min(self.MAX_LINEAR_SPEED, self.LINEAR_SPEED * per_dist), 1 )

        self.filtered_angular_z = self.low_pass_filter(self.filtered_angular_z, twist.angular.z, alpha=0.3)

        twist.angular.z = self.filtered_angular_z

        self.last_heading_error = heading_error
        self.last_time = current_time
        self.time_data.append(current_time - self.start_time)
        self.angular_data.append(self.current_theta)

        return twist,dist_to_goal

    def smooth_data(self, data, window_size=5):
        if len(data) < window_size:
            return np.array(data)
        return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

    def plot_combined(self):
        fig = plt.figure(figsize=(10, 10))
        gs = plt.GridSpec(2, 2)

        ax1 = fig.add_subplot(gs[0, 0])
        min_len_linear = min(len(self.time_data), len(self.linear_velocity_data))
        ax1.plot(self.time_data[:min_len_linear], self.linear_velocity_data[:min_len_linear], label="Linear Velocity (m/s)", color='blue')
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Velocity (m/s)")
        ax1.set_title("Linear Velocity")
        ax1.grid(True)
        ax1.legend()

        ax2 = fig.add_subplot(gs[0, 1])
        if len(self.x_data) > 5 and len(self.y_data) > 5:
            x_position = self.smooth_data(self.x_data)
            y_position = self.smooth_data(self.y_data)
            min_len_xy = min(len(x_position), len(y_position))
            x_position = x_position[:min_len_xy]
            y_position = y_position[:min_len_xy]
        else:
            x_position = self.x_data
            y_position = self.y_data

        ax2.plot(x_position, y_position, 'r-', label="Position")
        ax2.set_xlabel("X (m)")
        ax2.set_ylabel("Y (m)")
        ax2.set_title("Position")
        ax2.grid(True)
        ax2.legend()

        ax3 = fig.add_subplot(gs[1, 1])
        min_len_yaw = min(len(self.time_data), len(self.angular_data), len(self.target_heading_data))
        yaw_unwrapped = np.unwrap(self.angular_data[:min_len_yaw])
        yaw_deg_unwrapped = np.degrees(yaw_unwrapped)
        normalized_target = [self.normalize_angle(a) for a in self.target_heading_data[:min_len_yaw]]
        target_unwrapped = np.unwrap(normalized_target)
        target_deg_unwrapped = np.degrees(target_unwrapped)

        ax3.plot(self.time_data[:min_len_yaw], yaw_deg_unwrapped, label="YAW", color='g')
        ax3.plot(self.time_data[:min_len_yaw], target_deg_unwrapped, label="Target Heading", color='orange', linestyle='--')
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("YAW / Heading (degree)")
        ax3.set_title("YAW vs Target Heading")
        ax3.grid(True)
        ax3.legend()

        ax4 = fig.add_subplot(gs[1, 0])
        min_len_angular = min(len(self.time_data), len(self.angular_velocity_data))
        ax4.plot(self.time_data[:min_len_angular], self.angular_velocity_data[:min_len_angular], label="Angular Velocity (rad/s)", color='red')
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("Angular Velocity (rad/s)")
        ax4.set_title("Angular Velocity")
        ax4.grid(True)
        ax4.legend()

        plt.tight_layout()
        plt.show()

    def run(self):
        rate = rospy.Rate(10)
        arrived = False

        while not rospy.is_shutdown():
            if self.current_waypoint_index +1 >= len(self.waypoints):
                rospy.loginfo("Arrived at final destination. Stopping robot.")
                self.cmd_vel_pub.publish(Twist())
                arrived = True
                break

            previous_x,previous_y = self.waypoints[self.current_waypoint_index]
            goal_x, goal_y = self.waypoints[self.current_waypoint_index + 1]
            twist,dist = self.control_adaptive_los(goal_x, goal_y,previous_x,previous_y)
            self.linear_velocity_data.append(twist.linear.x)
            self.angular_velocity_data.append(twist.angular.z)

            if dist <= self.GOAL_RADIUS:
                rospy.loginfo("Reached waypoint ({}, {})".format(goal_x, goal_y))
                self.current_waypoint_index += 1

            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        if arrived:
            self.plot_combined()

if __name__ == '__main__':
    controller = AdaptiveLOSTrajectoryController()
    controller.run()
