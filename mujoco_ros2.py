#!/usr/bin/env python3
import sys
import time
import numpy as np

sys.path.insert(0, "/root/unitree_sdk2_python")

import mujoco
import mujoco.viewer as mujviewer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

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
        super().__init__('mujoco_ros2_stand')
        self.pub = self.create_publisher(JointState, '/go2/sim/joint_states', 10)
        self.sub = self.create_subscription(JointState, '/go2/sim/velocity_cmd', self.cmd_callback, 10)
        self.cmd_vel = np.zeros(12)

    def cmd_callback(self, msg):
        n = min(len(msg.velocity), 12)
        self.cmd_vel[:n] = np.array(msg.velocity[:n])

rclpy.init()
node = MuJoCoNode()

js_msg = JointState()
js_msg.name = [
    "FR_0","FR_1","FR_2",
    "FL_0","FL_1","FL_2",
    "RR_0","RR_1","RR_2",
    "RL_0","RL_1","RL_2"
]

target_pos = np.array([
    0.0, 0.735, -1.57,
    0.0, 0.735, -1.57,
    0.0, 0.735, -1.57,
    0.0, 0.735, -1.57
])

start_pos = data.qpos[7:].copy()
num_joints = len(start_pos)
percent = 0.0
duration = 500

viewer = mujviewer.launch_passive(model, data)

joint_indices = np.arange(6, 18)

while viewer.is_running():
    mujoco.mj_step(model, data)

    if percent < 1.0:
        percent += 1.0 / duration
        percent = min(percent, 1.0)
        for i in range(num_joints):
            idx = 7 + i
            data.qpos[idx] = (1 - percent) * start_pos[i] + percent * target_pos[i]

    for i, idx in enumerate(joint_indices):
        torque = Kp * (target_pos[i] - data.qpos[7 + i]) + Kd * (node.cmd_vel[i] - data.qvel[idx])
        data.qfrc_applied[idx] = torque

    js_msg.position = data.qpos[7:7+num_joints].tolist()
    js_msg.velocity = data.qvel[joint_indices].tolist()
    js_msg.header.stamp = node.get_clock().now().to_msg()
    node.pub.publish(js_msg)

    viewer.sync()
    time.sleep(0.002)

viewer.close()

