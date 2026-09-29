#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_recorder_node — Graba rosbag de la cámara durante el experimento.

Se suscribe a /experiment/running (Bool, TRANSIENT_LOCAL) publicado por
experiment_timer_node y arranca/detiene 'ros2 bag record' automáticamente.

Parámetros:
  bag_dir   str   Directorio donde se guardan los bags (default ~/bags)
  topics    list  Tópicos a grabar (default ['/image', '/camera_info'])

Uso típico (junto con camera_record.launch.py):
  ros2 launch qupa_experiment camera_record.launch.py
"""

import os
import subprocess
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool


class CameraRecorderNode(Node):

    def __init__(self):
        super().__init__('camera_recorder_node')

        self.declare_parameter('bag_dir', '~/bags')
        self.declare_parameter('topics', ['/image', '/camera_info'])

        self._bag_dir = self.get_parameter('bag_dir').value
        self._topics = self.get_parameter('topics').value
        self._proc = None

        _latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            Bool, '/experiment/running', self._running_cb, _latched_qos
        )

        self.get_logger().info(
            f'Camera recorder ready | bag_dir={self._bag_dir} | '
            f'topics={self._topics}'
        )

    def _running_cb(self, msg: Bool):
        if msg.data:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        if self._proc is not None:
            self.get_logger().warn('Recording already in progress, ignoring start.')
            return

        bag_dir = os.path.expanduser(self._bag_dir)
        os.makedirs(bag_dir, exist_ok=True)

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path = os.path.join(bag_dir, f'experiment_{stamp}')

        cmd = ['ros2', 'bag', 'record', '-o', bag_path] + list(self._topics)
        self._proc = subprocess.Popen(cmd)

        self.get_logger().info(f'Recording STARTED -> {bag_path}')

    def _stop_recording(self):
        if self._proc is None:
            return

        self._proc.terminate()
        try:
            self._proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('ros2 bag did not stop cleanly, killing it.')
            self._proc.kill()
            self._proc.wait()

        self._proc = None
        self.get_logger().info('Recording STOPPED')

    def destroy_node(self):
        self._stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
