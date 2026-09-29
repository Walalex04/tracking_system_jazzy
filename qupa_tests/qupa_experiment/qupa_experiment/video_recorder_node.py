#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video_recorder_node — Graba un archivo de video (.mp4) de la cámara durante el experimento.

Se suscribe a /experiment/running (Bool, TRANSIENT_LOCAL) publicado por
experiment_timer_node y a un tópico de imagen (por defecto 'image_rect', la
imagen ya rectificada pero SIN recortar). Mientras el experimento está
corriendo, escribe cada frame recibido a un archivo .mp4 con cv2.VideoWriter.

La cámara/pipeline puede entregar frames a una tasa real distinta (y variable)
de 'fps'. Para que la duración del video coincida con el tiempo real grabado,
los frames se escriben según el reloj de pared: si la cámara va más lenta que
'fps' se duplica el último frame para no acelerar el video; si va más rápida
se descartan frames sobrantes.

Máscara opcional: si 'apply_mask' es true, se aplica sobre cada frame la misma
máscara poligonal que usa cropper_node (leída directamente de 'mask_config_path',
el mismo config_cropper.yaml), ennegreciendo todo lo que quede fuera del área
de la arena. Si es false, se graba la imagen completa sin recortar.

Parámetros:
  video_dir        str    Directorio donde se guardan los videos (default ~/videos)
  image_topic      str    Tópico de imagen a grabar (default 'image_rect')
  fps              float  Cuadros por segundo del video de salida (default 30.0)
  apply_mask       bool   Si aplica la máscara de la arena (default False)
  mask_config_path str    Ruta al yaml de puntos (config_cropper.yaml), usado
                          solo si apply_mask=True

Uso típico (junto con camera_record_video.launch.py):
  ros2 launch qupa_experiment camera_record_video.launch.py apply_mask:=true
  ros2 service call /experiment/start std_srvs/srv/Trigger
  ros2 service call /experiment/stop  std_srvs/srv/Trigger
"""

import os
import time
import datetime

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VideoRecorderNode(Node):

    def __init__(self):
        super().__init__('video_recorder_node')

        self.declare_parameter('video_dir', '~/videos')
        self.declare_parameter('image_topic', 'image_rect')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('apply_mask', False)
        self.declare_parameter('mask_config_path', '')

        self._video_dir = self.get_parameter('video_dir').value
        self._image_topic = self.get_parameter('image_topic').value
        self._fps = self.get_parameter('fps').value
        self._apply_mask = self.get_parameter('apply_mask').value
        self._mask_config_path = self.get_parameter('mask_config_path').value

        self._bridge = CvBridge()
        self._writer = None
        self._recording = False
        self._start_time = None
        self._frames_written = 0
        self._frames_received = 0

        self._mask = None
        self._vertices = []
        if self._apply_mask:
            self._vertices = self._load_mask_points(self._mask_config_path)
            if len(self._vertices) < 3:
                self.get_logger().error(
                    f'apply_mask=true pero solo se cargaron {len(self._vertices)} '
                    f'puntos (<3) desde "{self._mask_config_path}". '
                    'Se grabará SIN máscara (arena completa).'
                )
                self._apply_mask = False

        _latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Bool, '/experiment/running', self._running_cb, _latched_qos
        )

        _image_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Image, self._image_topic, self._image_cb, _image_qos
        )

        self.get_logger().info(
            f'Video recorder ready | video_dir={self._video_dir} | '
            f'image_topic={self._image_topic} | fps={self._fps} | '
            f'apply_mask={self._apply_mask}'
        )

    def _load_mask_points(self, path: str):
        if not path or not os.path.isfile(path):
            self.get_logger().error(f'mask_config_path inválido o no existe: "{path}"')
            return []

        with open(path, 'r') as f:
            config = yaml.safe_load(f) or {}

        params = config.get('/**', {}).get('ros__parameters', {})

        vertices = []
        i = 0
        while True:
            point = params.get(f'point_{i}')
            if point is None:
                break
            vertices.append((int(point[0]), int(point[1])))
            i += 1
        return vertices

    def _apply_arena_mask(self, frame):
        if self._mask is None:
            self._mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            cv2.fillPoly(self._mask, [np.array(self._vertices, dtype=np.int32)], 255)

        masked = np.zeros_like(frame)
        cv2.copyTo(frame, self._mask, masked)
        return masked

    def _running_cb(self, msg: Bool):
        if msg.data:
            self._recording = True
        else:
            self._recording = False
            self._close_writer()

    def _image_cb(self, msg: Image):
        if not self._recording:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        if self._apply_mask:
            frame = self._apply_arena_mask(frame)

        if self._writer is None:
            self._open_writer(frame.shape[1], frame.shape[0])

        self._frames_received += 1

        # Escribir según el reloj de pared, no 1 frame por callback: si la
        # cámara entrega más lento que self._fps, se duplica este frame las
        # veces necesarias para no acelerar el video; si entrega más rápido,
        # el frame se descarta (ya se alcanzó el conteo esperado).
        elapsed = time.monotonic() - self._start_time
        target_frame_count = int(elapsed * self._fps) + 1
        while self._frames_written < target_frame_count:
            self._writer.write(frame)
            self._frames_written += 1

    def _open_writer(self, width: int, height: int):
        video_dir = os.path.expanduser(self._video_dir)
        os.makedirs(video_dir, exist_ok=True)

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        video_path = os.path.join(video_dir, f'experiment_{stamp}.mp4')

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self._writer = cv2.VideoWriter(video_path, fourcc, self._fps, (width, height))
        self._start_time = time.monotonic()
        self._frames_written = 0
        self._frames_received = 0

        self.get_logger().info(f'Recording STARTED -> {video_path}')

    def _close_writer(self):
        if self._writer is None:
            return

        elapsed = time.monotonic() - self._start_time
        input_fps = self._frames_received / elapsed if elapsed > 0 else 0.0
        self._writer.release()
        self._writer = None
        self.get_logger().info(
            f'Recording STOPPED | duration={elapsed:.1f}s | '
            f'input~{input_fps:.1f}fps | output_frames={self._frames_written}'
        )

    def destroy_node(self):
        self._close_writer()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
