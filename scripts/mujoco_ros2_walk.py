#!/usr/bin/env python3
import sys
import time
import numpy as np
import os

# --- Graphics Fix ---
os.environ['__GL_NO_MIT_SHM'] = '1'

sys.path.insert(0, "/root/unitree_sdk2_python")

import mujoco
import mujoco.viewer as mujviewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

XML_PATH = "/root/unitree_mujoco/unitree_robots/go2/scene.xml"
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
data.qpos[2] = 0.38

model.opt.gravity[:] = [0.0, 0.0, -9.81]

Kp = 80.0
Kd = 8.0

class MuJoCoNode(Node):
    def __init__(self):
        super().__init__('mujoco_ros2_walk')
        self.pub = self.create_publisher(JointState, '/go2/sim/joint_states', 10)
        # publish twist from terminal for now
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.cmd_vel = Twist() 

    def cmd_callback(self, msg):
        self.cmd_vel = msg

rclpy.init()
node = MuJoCoNode()

js_msg = JointState()
js_msg.name = [
    "FR_0","FR_1","FR_2",
    "FL_0","FL_1","FL_2", 
    "RR_0","RR_1","RR_2", 
    "RL_0","RL_1","RL_2"
]
num_joints = len(js_msg.name) 

# initial
STAND_POS = np.array([
    0.0, 0.735, -1.57,  # FR
    0.0, 0.735, -1.57,  # FL
    0.0, 0.735, -1.57,  # RR
    0.0, 0.735, -1.57   # RL
])

GAIT_FREQ = 4.0        # steps per second (Hz)
STEP_HEIGHT = 0.08     # feet height
STEP_LENGTH_FACTOR = 0.6 # swing leg based on cmd_vel
TURN_FACTOR = 0.4      # yaw hips based on cmd_vel

start_pos = data.qpos[7:].copy()
percent = 0.0
# how fast start the standing
duration = 200 

target_pos = start_pos.copy() 

viewer = mujviewer.launch_passive(model, data)
joint_indices = np.arange(6, 18) #joints 7-18

start_time = time.time()

try:
    while viewer.is_running():
        current_time = time.time()
    
        rclpy.spin_once(node, timeout_sec=0.001)
        mujoco.mj_step(model, data)

        
        if percent < 1.0:
            percent += 1.0 / duration
            percent = min(percent, 1.0)
            target_pos = (1 - percent) * start_pos + percent * STAND_POS
        else:
            
            linear_cmd = node.cmd_vel.linear.x
            angular_cmd = node.cmd_vel.angular.z

            gait_clock = (current_time - start_time) * GAIT_FREQ
            
            phase_1 = (gait_clock * 2 * np.pi)
            phase_2 = (gait_clock * 2 * np.pi) + np.pi

            s1 = np.sin(phase_1)
            c1 = np.cos(phase_1)
            s2 = np.sin(phase_2)
            c2 = np.cos(phase_2)

            offsets = np.zeros(12)

            
            
            
            z_swing_1 = STEP_HEIGHT * max(0, s1)
            
            x_swing_1 = -STEP_LENGTH_FACTOR * c1 * linear_cmd 
            
            
            offsets[1] += x_swing_1  # Thigh
            offsets[2] += 2.0 * z_swing_1        # Calf
            offsets[10] += x_swing_1 # (RL Thigh)
            offsets[11] += 2.0 * z_swing_1       # (RL Calf)
            
            
            z_swing_2 = STEP_HEIGHT * max(0, s2)
            x_swing_2 = -STEP_LENGTH_FACTOR * c2 * linear_cmd
            
            
            offsets[4] += x_swing_2  # (FL Thigh)
            offsets[5] += 2.0 * z_swing_2        # (FL Calf)
            offsets[7] += x_swing_2  # (RR Thigh) 
            offsets[8] += 2.0 * z_swing_2        # (RR Calf)
            

        
            turn_offset = TURN_FACTOR * angular_cmd
            offsets[0] = -turn_offset  # FR Hip
            offsets[3] = turn_offset   # FL Hip
            offsets[6] = -turn_offset  # RR Hip
            offsets[9] = turn_offset   # RL Hip

            target_pos = STAND_POS + offsets

        # PD Controller
        for i, idx in enumerate(joint_indices):
            pos_error = target_pos[i] - data.qpos[7 + i]
            vel_error = 0.0 - data.qvel[idx] 
            torque = Kp * pos_error + Kd * vel_error
            data.qfrc_applied[idx] = torque

        #  Pub joint states
        js_msg.position = data.qpos[7:7+num_joints].tolist()
        js_msg.velocity = data.qvel[joint_indices].tolist()
        js_msg.header.stamp = node.get_clock().now().to_msg()
        node.pub.publish(js_msg)

        
        viewer.sync()
        time.sleep(0.002)

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    viewer.close()
    node.destroy_node()
    rclpy.shutdown()


