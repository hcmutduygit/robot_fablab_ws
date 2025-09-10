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
from scipy.interpolate import interp1d

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
        target_heading = 0.0
# 0,41 0.52
        self.KP = 5
        self.LINEAR_SPEED = 1.2
        self.ANGULAR_SPEED = 1
        self.GOAL_RADIUS = 0.5
        self.LOOKAHEAD_DISTANCE = 0.2
        self.MAX_LINEAR_SPEED = 1.5
        self.MAX_ANGULAR_SPEED = 1.5

        # self.waypoints = [(0,0),(7.765,-4.42),(19.68,-12.1),(28.912,3.44),(35.22, 12.78),(39.6, 10.324),(50.53,2.95)] #Uni->Bus
        # self.waypoints = [(0,0),(7.765,-4.42),(19.68,-12.1),(28.912,3.44),(35.22, 12.78),(45.6798,29.6695),(41.0207 , 36.3534)]
        # self.waypoints = [(0,0),(10,0),(10,10),(0,10),(0,0)]
        self.waypoints = [(0,0),(8,0),(8,6.5),(25,6.5),(25,24),(30,24)]
        # self.waypoints = [(0,0),(6,0),(6,-6),(0,-6),(0,0)]

        self.x_data = []
        self.y_data = []
        self.cte_data = []
        self.angular_data = []
        self.time_data = []
        self.start_time = time.time()
        self.target_heading_data = []


        self.current_waypoint_index = 0

        self.waypoints = self.interpolate_waypoints(self.waypoints)

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

    def euler_from_quaternion(self, x, y, z, w):
        return tft.euler_from_quaternion([x, y, z, w])

    def distance1(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def get_heading(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def get_lookahead_point(self, x, y, theta, distance):
        return x + distance * math.cos(theta), y + distance * math.sin(theta)

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def interpolate_waypoints(self, waypoints, resolution=1, remove_old_waypoints=False):
        new_waypoints = []
        for i in range(len(waypoints) - 1):
            start_x, start_y = waypoints[i]
            end_x, end_y = waypoints[i + 1]
            dist = self.distance1(start_x, start_y, end_x, end_y)
            num_points = int(dist / resolution)
            for j in range(num_points):
                ratio = j / float(num_points)
                interp_x = start_x + ratio * (end_x - start_x)
                interp_y = start_y + ratio * (end_y - start_y)
                new_waypoints.append((interp_x, interp_y))
        new_waypoints.append(waypoints[-1])
        return [waypoints[0]] + new_waypoints[1:-1] + [waypoints[-1]] if remove_old_waypoints else new_waypoints
    def distance(self, p1, p2):
        return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    # def interpolate_waypoints_with_fillets(self, waypoints, fillet_radius=0.1, resolution=0.5):
    #     if len(waypoints) < 2:
    #         return waypoints

    #     smoothed_path = []
        
    #     temp_waypoints = list(waypoints)
    #     is_closed_loop = (len(waypoints) > 2 and waypoints[0] == waypoints[-1])
    #     if not is_closed_loop: 
    #         temp_waypoints.insert(0, waypoints[0]) 
    #         temp_waypoints.append(waypoints[-1])

    #     for i in range(1, len(temp_waypoints) - 1):
    #         p_prev = temp_waypoints[i-1] 
    #         p_curr = temp_waypoints[i]   
    #         p_next = temp_waypoints[i+1] 

    #         if i == 1 and not is_closed_loop:
    #             smoothed_path.append(p_prev)

    #         v_prev = np.array(p_prev) - np.array(p_curr)
    #         v_next = np.array(p_next) - np.array(p_curr)

    #         norm_v_prev = np.linalg.norm(v_prev)
    #         norm_v_next = np.linalg.norm(v_next)
            
    #         if norm_v_prev == 0 or norm_v_next == 0:
    #             continue 
            
    #         unit_v_prev = v_prev / norm_v_prev
    #         unit_v_next = v_next / norm_v_next

    #         dot_product = np.dot(unit_v_prev, unit_v_next)
    #         angle_rad = np.arccos(np.clip(dot_product, -1.0, 1.0))
    #         if np.isclose(angle_rad, np.pi) or np.isclose(angle_rad, 0): 
    #             smoothed_path.append(p_curr)
    #             continue

    #         max_fillet_dist_prev = norm_v_prev / 2.0
    #         max_fillet_dist_next = norm_v_next / 2.0
            
    #         current_fillet_radius = min(fillet_radius, max_fillet_dist_prev, max_fillet_dist_next)

    #         arc_start_point = np.array(p_curr) + unit_v_prev * current_fillet_radius
            
    #         arc_end_point = np.array(p_curr) + unit_v_next * current_fillet_radius
    #         if len(smoothed_path) > 0 and smoothed_path[-1] != tuple(arc_start_point): 
    #             p_seg_start = smoothed_path[-1]
    #             p_seg_end = tuple(arc_start_point)
                
    #             dist_seg = self.distance(p_seg_start, p_seg_end)
    #             num_seg_points = max(2, int(dist_seg / resolution))
                
    #             if num_seg_points > 1:
    #                 for j in range(1, num_seg_points): 
    #                     ratio = j / float(num_seg_points -1 )
    #                     interp_x = p_seg_start[0] + ratio * (p_seg_end[0] - p_seg_start[0])
    #                     interp_y = p_seg_start[1] + ratio * (p_seg_end[1] - p_seg_start[1])
    #                     smoothed_path.append((interp_x, interp_y))
    #             else: 
    #                 smoothed_path.append(p_seg_end)
    #         elif len(smoothed_path) == 0: 
    #              smoothed_path.append(p_prev)
    #              p_seg_start = p_prev
    #              p_seg_end = tuple(arc_start_point)
    #              dist_seg = self.distance(p_seg_start, p_seg_end)
    #              num_seg_points = max(2, int(dist_seg / resolution))
    #              if num_seg_points > 1:
    #                  for j in range(1, num_seg_points):
    #                      ratio = j / float(num_seg_points-1)
    #                      interp_x = p_seg_start[0] + ratio * (p_seg_end[0] - p_seg_start[0])
    #                      interp_y = p_seg_start[1] + ratio * (p_seg_end[1] - p_seg_start[1])
    #                      smoothed_path.append((interp_x, interp_y))
    #              else:
    #                  smoothed_path.append(p_seg_end)
    #         normal_prev = np.array([-unit_v_prev[1], unit_v_prev[0]])

    #         bisector_dir = (unit_v_prev + unit_v_next) / np.linalg.norm(unit_v_prev + unit_v_next)
            
    #         alpha = np.arccos(np.dot(unit_v_prev, unit_v_next)) 
            
    #         cross_product = unit_v_prev[0] * unit_v_next[1] - unit_v_prev[1] * unit_v_next[0]
            
    #         center_dist_from_corner = current_fillet_radius / np.tan(alpha / 2.0)
            
    #         center_offset_dir = (unit_v_prev + unit_v_next) / np.linalg.norm(unit_v_prev + unit_v_next)
    #         center = np.array(p_curr) + center_offset_dir * center_dist_from_corner
            
    #         start_angle = np.arctan2(arc_start_point[1] - center[1], arc_start_point[0] - center[0])
    #         end_angle = np.arctan2(arc_end_point[1] - center[1], arc_end_point[0] - center[0])

    #         if cross_product < 0:
    #             if end_angle > start_angle:
    #                 end_angle -= 2 * np.pi
    #         else: 
    #             if end_angle < start_angle:
    #                 end_angle += 2 * np.pi

    #         num_arc_points = max(2, int(current_fillet_radius * abs(end_angle - start_angle) / resolution))
    #         if num_arc_points > 1:
    #             arc_angles = np.linspace(start_angle, end_angle, num_arc_points)
    #             for angle in arc_angles:
    #                 arc_x = center[0] + current_fillet_radius * np.cos(angle)
    #                 arc_y = center[1] + current_fillet_radius * np.sin(angle)
    #                 smoothed_path.append((arc_x, arc_y))
    #         else:
    #             smoothed_path.append(tuple(arc_start_point))
    #             smoothed_path.append(tuple(arc_end_point))


    #     if not is_closed_loop:
    #         p_seg_start = smoothed_path[-1]
    #         p_seg_end = temp_waypoints[-1]
            
    #         dist_seg = self.distance(p_seg_start, p_seg_end)
    #         num_seg_points = max(2, int(dist_seg / resolution))
            
    #         if num_seg_points > 1:
    #             for j in range(1, num_seg_points +1):
    #                 ratio = j / float(num_seg_points)
    #                 interp_x = p_seg_start[0] + ratio * (p_seg_end[0] - p_seg_start[0])
    #                 interp_y = p_seg_start[1] + ratio * (p_seg_end[1] - p_seg_start[1])
    #                 smoothed_path.append((interp_x, interp_y))
    #         else:
    #             smoothed_path.append(p_seg_end)
    #     else:
    #         if smoothed_path[-1] != smoothed_path[0]:
    #             smoothed_path.append(smoothed_path[0])
    #     return smoothed_path

    def get_cross_track_error(self, px, py, ax, ay, bx, by):
        dx = bx - ax
        dy = by - ay
        apx = px - ax
        apy = py - ay
        cross = dx * apy - dy * apx
        return cross

    def control_adaptive_los(self, goal_x, goal_y):
        x_fake, y_fake = self.get_lookahead_point(self.current_x, self.current_y, self.current_theta, self.LOOKAHEAD_DISTANCE)

        twist = Twist()
        target_heading = self.get_heading(x_fake, y_fake, goal_x, goal_y)
        self.target_heading_data.append(target_heading)
        heading_error = self.normalize_angle(target_heading - self.current_theta)

        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 0.1
        derivative = (heading_error - self.last_heading_error) / dt if dt > 0 else 0.0
        # angular_z = self.ANGULAR_SPEED * heading_error
        angular_z = (self.ANGULAR_SPEED * heading_error + self.KP * derivative)
        angular_z = max(min(angular_z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        twist.angular.z = angular_z

        dist_to_goal = self.distance1(self.current_x, self.current_y, goal_x, goal_y)
        if abs(heading_error) > math.pi / 4:
            twist.linear.x = max(min(self.MAX_LINEAR_SPEED / 2, self.LINEAR_SPEED * dist_to_goal), 0.05)
        else:
            twist.linear.x = max(min(self.MAX_LINEAR_SPEED, self.LINEAR_SPEED * dist_to_goal), 0.05)

        self.last_heading_error = heading_error
        self.last_time = current_time
        self.angular_data.append(self.current_theta)

        if self.current_waypoint_index < len(self.waypoints) - 1:
            ax, ay = self.waypoints[self.current_waypoint_index]
            bx, by = self.waypoints[self.current_waypoint_index + 1]
            cte = self.get_cross_track_error(self.current_x, self.current_y, ax, ay, bx, by)
            self.cte_data.append(cte)
            self.time_data.append(current_time - self.start_time)

        return twist

    def smooth_data(self, data, window_size=5):
        if len(data) < window_size:
            return np.array(data)
        return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

    def plot_combined(self):
        fig = plt.figure(figsize=(10, 10))
        gs = plt.GridSpec(2, 2)

        ax1 = fig.add_subplot(gs[0, 1])
        if len(self.x_data) > 5 and len(self.y_data) > 5:
            x_position = self.smooth_data(self.x_data)
            y_position = self.smooth_data(self.y_data)
            min_len_xy = min(len(x_position), len(y_position))
            x_position = x_position[:min_len_xy]
            y_position = y_position[:min_len_xy]
        else:
            x_position = self.x_data
            y_position = self.y_data

        ax1.plot(x_position, y_position, 'r-', label="Position")
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_title("Position")
        ax1.grid(True)
        ax1.legend()

        ax2 = fig.add_subplot(gs[0, 0])
        min_len_cte = min(len(self.time_data), len(self.cte_data))
        ax2.plot(self.time_data[:min_len_cte], self.cte_data[:min_len_cte], label="CTE (Cross Track Error)")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("CTE (m)")
        ax2.set_title("CTE (Cross Track Error)")
        ax2.grid(True)
        ax2.legend()

        ax3 = fig.add_subplot(gs[1, 0])
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

        plt.tight_layout()
        plt.show()

    def run(self):
        rate = rospy.Rate(100)
        arrived = False

        while not rospy.is_shutdown():
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("Arrived at final destination. Stopping robot.")
                self.cmd_vel_pub.publish(Twist())
                arrived = True
                break

            goal_x, goal_y = self.waypoints[self.current_waypoint_index]
            dist = self.distance1(self.current_x, self.current_y, goal_x, goal_y)
            twist = self.control_adaptive_los(goal_x, goal_y)

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