import os
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import scipy.spatial.transform as st
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QFileDialog
from PyQt6.QtGui import QPixmap, QImage, QMouseEvent
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2

from robot_workspaces.mj_transporter_franka_table import FrankaTable
import envlogger

class ImageSubscriber(QThread):
    new_image = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.cv_bridge = CvBridge()

    def run(self):
        node = rclpy.create_node('image_subscriber')
        subscription = node.create_subscription(Image, 'overhead_camera', self.image_callback, 10)
        rclpy.spin(node)

    def image_callback(self, msg):
        rgb_img = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        self.new_image.emit(rgb_img)


class MainWindow(QMainWindow):
    def __init__(self, env):
        super().__init__()

        # initialize the GUI
        self.initUI()

        # start ROS image subscriber
        self.image_subscriber = ImageSubscriber()
        self.image_subscriber.new_image.connect(self.update_image)
        self.image_subscriber.start()

        # environment for recording data
        self.env = env

        # GUI application parameters
        self.mode = "pick"
        self.table_height = 0.0
        self.x = 0.0
        self.y = 0.0
        self.gripper_rot_z = 0.0
        self.camera_intrinsics = None
        self.camera_extrinsics = None

    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_screen = QVBoxLayout(central_widget)
        horizontal_panes = QHBoxLayout()
        left_pane = QVBoxLayout()
        right_pane = QVBoxLayout()
        right_pane.setSpacing(2)

        # Left Pane
        self.label_image = QLabel()
        self.label_image.mousePressEvent = self.update_pixel_coordinates
        left_pane.addWidget(self.label_image)
        horizontal_panes.addLayout(left_pane)


        # Right Pane
        # table height input
        self.label_table_height = QLabel("Enter Table Height Offset:")
        self.label_table_height.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.label_table_height.setFixedHeight(20)
        self.line_edit = QLineEdit()
        self.line_edit.setPlaceholderText("Enter table height")
        self.line_edit.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.line_edit.setFixedHeight(20)
        self.line_edit.returnPressed.connect(self.update_application_params)

        # pixel coordinates
        self.label_pixel_coords = QLabel("Selected Pixel Coordinates:")
        self.label_pixel_coords.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.label_pixel_coords.setFixedHeight(20)
            
        pixel_coords_box = QHBoxLayout()
        
        self.x_edit = QLineEdit()
        self.x_edit.setPlaceholderText("X")
        self.x_edit.returnPressed.connect(self.update_pixel_coordinates)
        self.x_edit.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.x_edit.setFixedHeight(20)

        self.y_edit = QLineEdit()
        self.y_edit.setPlaceholderText("Y")
        self.y_edit.returnPressed.connect(self.update_pixel_coordinates)
        self.y_edit.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.y_edit.setFixedHeight(20)

        pixel_coords_box.addWidget(self.x_edit)
        pixel_coords_box.addWidget(self.y_edit)
    
        # gripper orientation input
        self.label_gripper_orientation = QLabel("Enter Gripper Rotation:")
        self.label_gripper_orientation.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.label_gripper_orientation.setFixedHeight(20)

        self.gripper_rot_z_edit = QLineEdit()
        self.gripper_rot_z_edit.setPlaceholderText("Z")
        self.gripper_rot_z_edit.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.gripper_rot_z_edit.setFixedHeight(20)
        
        # button to upload file containing camera calibration parameters
        self.upload_button = QPushButton("Upload Camera Parameters")
        self.upload_button.setFixedHeight(20)
        self.upload_button.clicked.connect(self.upload_camera_params)

        # button to update application parameters
        self.submit_button = QPushButton("Submit Application Parameters")
        self.submit_button.clicked.connect(self.update_application_params)
        self.submit_button.setFixedHeight(20)
        
        # step environment button
        self.step_button = QPushButton("Step Environment")
        self.step_button.clicked.connect(self.env_step)
        self.step_button.setFixedHeight(20)
    
        # reset environment button
        self.reset_button = QPushButton("Reset Environment")
        self.reset_button.clicked.connect(self.env_reset)
        self.reset_button.setFixedHeight(20)

        # add all widgets to right pane
        right_pane.addWidget(self.label_table_height)
        right_pane.addWidget(self.line_edit)
        right_pane.addWidget(self.label_pixel_coords)
        right_pane.addLayout(pixel_coords_box)
        right_pane.addWidget(self.label_gripper_orientation)
        right_pane.addWidget(self.gripper_rot_z_edit)
        right_pane.addWidget(self.submit_button)
        right_pane.addWidget(self.upload_button)
        right_pane.addWidget(self.step_button)
        right_pane.addWidget(self.reset_button)

        # add to main screen
        horizontal_panes.addLayout(right_pane)
        main_screen.addLayout(horizontal_panes)
        central_widget.setLayout(main_screen)

        self.setWindowTitle('Transporter Network Data Collection')
        self.show()

    def update_pixel_coordinates(self, event: QMouseEvent):
        point = event.pos()
        x = point.x()
        y = point.y()
        self.x_edit.setText(str(x))
        self.y_edit.setText(str(y))

    def upload_camera_params(self):
        """
        Reads camera calibration parameters from a json file
        """
        fileName, _ = QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","All Files (*);;Python Files (*.py)")
        if fileName:
            print("File selected:", fileName)
            import json
            with open(fileName) as f:
                data = json.load(f)
                
            # assign camera intrinsics
            fx = data["intrinsic_params"]["fx"]
            fy = data["intrinsic_params"]["fy"]
            cx = data["intrinsic_params"]["cx"]
            cy = data["intrinsic_params"]["cy"]
            self.camera_intrinsics = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
            
            # assign camera extrinsics
            translation = data["extrinsic_params"]["translation"]
            quaternion = data["extrinsic_params"]["quaternion"]
            rotation = st.Rotation.from_quat(quaternion).as_matrix()
            self.camera_extrinsics = np.eye(4)
            self.camera_extrinsics[:3, :3] = rotation
            self.camera_extrinsics[:3, 3] = translation
            
            print("Camera Intrinsics:", self.camera_intrinsics)
            print("Camera Extrinsics:", self.camera_extrinsics)


    def update_application_params(self):
        self.table_height = float(self.line_edit.text())  # Convert input text to float 
        self.x = float(self.x_edit.text())
        self.y = float(self.y_edit.text())
        self.gripper_rot_z = float(self.gripper_rot_z_edit.text())
        
        print("Updating Table Height:", self.table_height)
        print("Updating X Coordinate:", self.x)
        print("Updating Y Coordinate:", self.y)     
        print("Updating Gripper Rotation:", self.gripper_rot_z)

    def overlay_gripper_line(self, image, center, length, angle_degrees, color=(255, 0, 0), thickness=3):
        # Convert angle from degrees to radians
        angle_radians = math.radians(angle_degrees)

        # Calculate start and endpoint of the line
        x1 = int(center[0] - (length/2) * math.cos(angle_radians))
        y1 = int(center[1] - (length/2) * math.sin(angle_radians))
        x2 = int(center[0] + (length/2) * math.cos(angle_radians))
        y2 = int(center[1] + (length/2) * math.sin(angle_radians))

        # Draw the line on the image
        cv2.line(image, (x1, y1), (x2, y2), color, thickness)

    def update_image(self, rgb_img):
        # store the current image
        self.current_image = rgb_img.copy() 

        # overlay a line centred at pixel coord with gripper orientation
        x = int(self.x)
        y = int(self.y)
        gripper_rot_z = self.gripper_rot_z
        self.overlay_gripper_line(rgb_img, (x, y), 50, gripper_rot_z)

        # display the image
        height, width, channel = rgb_img.shape
        bytes_per_line = channel * width
        qimg = QImage(rgb_img.data, width, height, QImage.Format(13))
        pixmap = QPixmap.fromImage(qimg)
        self.label_image.setPixmap(pixmap)
    

    def env_step(self):
        # map pixel coords to world coords
        ## TODO: get depth value from camera 
        depth_val = 0.4

        ## convert current pixels coordinates to camera frame coordinates
        pixel_coords = np.array([self.x, self.y])
        image_coords = np.concatenate([pixel_coords, np.ones(1)])
        camera_coords =  np.linalg.inv(self.camera_intrinsics) @ image_coords
        camera_coords *= -depth_val # negative sign due to mujoco camera convention (for debug only!)

        ## convert camera coordinates to world coordinates
        camera_coords = np.concatenate([camera_coords, np.ones(1)])
        world_coords = np.linalg.inv(self.camera_extrinsics) @ camera_coords
        world_coords = world_coords[:3] / world_coords[3]

        print("World Coordinates:", world_coords)
        
        world_coords = np.array([0.25, 0.25, 0.5]) # hardcode while debugging
        quat = R.from_euler('xyz', [0, 180, self.gripper_rot_z], degrees=True).as_quat()
        pose = np.concatenate([world_coords, quat])

        if self.mode == "pick":
            self.env.step(pose)
        else:
            self.env.step(pose)
    
    def env_reset(self):
        self.env.set_observation(self.current_image)
        self.env.reset()


def main(args=None):
    rclpy.init(args=None)
    env = FrankaTable()
    os.makedirs(os.path.join(os.path.dirname(__file__), "data"), exist_ok=True)
    with envlogger.EnvLogger(
            env, 
            data_directory=os.path.join(os.path.dirname(__file__), "data")) as env:
        app = QApplication(sys.argv)
        ex = MainWindow(env)
        sys.exit(app.exec())

if __name__ == '__main__':
    main()

