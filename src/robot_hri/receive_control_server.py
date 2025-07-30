#!/usr/bin/env python3

from flask import Flask, request, jsonify
import rospy
from std_msgs.msg import String
import threading

# --- ROS khởi tạo trong thread riêng để không block Flask ---
def ros_init():
    rospy.init_node('flask_control_node', anonymous=True)
    
ros_thread = threading.Thread(target=ros_init)
ros_thread.start()

# --- ROS Publisher ---
pub = rospy.Publisher('/goto_point', String, queue_size=10)

# --- Flask App ---
app = Flask(__name__)

@app.route('/move', methods=['POST'])
def move():
    data = request.get_json()
    destination = data.get("destination")

    if not destination:
        return jsonify({"status": "error", "message": "No destination"}), 400

    # Gửi lệnh tới topic
    pub.publish(destination)
    rospy.loginfo(f"[Flask] Sending robot to: {destination}")

    return jsonify({"status": "ok", "message": f"Going to {destination}"})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
