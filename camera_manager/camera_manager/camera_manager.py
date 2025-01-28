import os
from collections import namedtuple

import rclpy
import cv2
from rclpy.node import Node
from rclpy.node import HIDDEN_NODE_PREFIX

NodeName = namedtuple('NodeName', ('name', 'namespace', 'full_name'))
MAX_CAMERAS = 10


class CameraManager(Node):

    def __init__(self):
        super().__init__('camera_manager')
        self.avaiable = []
        self.cameras = []
        self.load_camera()
        print(self.avaiable)
        for i, cam_id in enumerate(self.avaiable):
            self.create_camera_node(f"Camera{i}", cam_id)


    def create_camera_node(self, name: str, id: int, width: int = 640, height: int = 480):
        if f"camera{name}" in self.cameras:
            raise RuntimeError(f"Camera {name} already exists")
        os.system(f'''
            ros2 run v4l2_camera v4l2_camera_node \\
            --ros-args \\
            -p image_size:="[{width},{height}]" \\
            -p camera_frame_id:=camera_optical_link \\
            -p video_device:="/dev/video{id}" \\
            -r __ns:="/{name}" & 
        ''')
        self.cameras.append(f"camera{name}")

    def load_camera(self):
        self.avaiable = []
        for i in range(MAX_CAMERAS):
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)

            if not cap.read()[0]:
                continue

            self.avaiable.append(i)
            cap.release()

def main(args=None):
    rclpy.init(args=args)
    manager = CameraManager()

    rclpy.spin(manager)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
