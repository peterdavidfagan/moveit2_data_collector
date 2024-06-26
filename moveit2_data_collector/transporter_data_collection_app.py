import os
import sys
import argparse
import yaml
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import griddata
import scipy.spatial.transform as st
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6.QtCore import *

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import message_filters

import cv2

from robot_workspaces.franka_table import FrankaTable
import envlogger
from envlogger.backends import tfds_backend_writer
import tensorflow as tf
import tensorflow_datasets as tfds


class Worker(QRunnable):

    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()
        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    @pyqtSlot()
    def run(self):
        self.fn(*self.args, **self.kwargs)

class ImageSubscriber(QThread):
    new_rgb_image = pyqtSignal(object)
    new_depth_image = pyqtSignal(object)

    def __init__(self, rgb_topic, depth_topic):
        super().__init__()
        self.cv_bridge = CvBridge()
        self.camera_callback_group = ReentrantCallbackGroup()
        self.camera_qos_profile = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy(rclpy.qos.HistoryPolicy.KEEP_LAST),
                reliability=QoSReliabilityPolicy(rclpy.qos.ReliabilityPolicy.RELIABLE),
            )
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic

    def __del__(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()

    def run(self):
        self.node = rclpy.create_node('image_subscriber')

        self.rgb_image_sub = message_filters.Subscriber(
            self.node,
            Image,
            self.rgb_topic,
            callback_group=self.camera_callback_group,
        )

        self.depth_image_sub = message_filters.Subscriber(
            self.node,
            Image,
            self.depth_topic,
            callback_group=self.camera_callback_group,
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_image_sub, self.depth_image_sub],
            10,
            0.1,
            )
        self.sync.registerCallback(self.image_callback)
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.spin()
       
    def update_rgb_topic(self, camera_topic):
        self.rgb_topic = camera_topic
        if (self.rgb_topic is not None) and (self.depth_topic is not None):
            self.executor.remove_node(self.node)
            self.node.destroy_subscription(self.rgb_image_sub)
            self.rgb_image_sub = message_filters.Subscriber(
                node,
                Image,
                self.rgb_topic,
                callback_group=self.camera_callback_group,
            )
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.rgb_image_sub, self.depth_image_sub],
                10,
                0.1,
                )
            self.sync.registerCallback(self.image_callback)
            self.executor.add_node(self.node)
            self.executor.wake()

    def update_depth_topic(self, camera_topic):
        self.depth_topic = camera_topic
        if (self.rgb_topic is not None) and (self.depth_topic is not None):
            self.executor.remove_node(self.node)
            self.node.destroy_subscription(self.rgb_image_sub)
            self.depth_image_sub = message_filters.Subscriber(
                node,
                Image,
                self.depth_topic,
                callback_group=self.camera_callback_group,
            )
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.rgb_image_sub, self.depth_image_sub],
                10,
                0.5,
                )
            self.sync.registerCallback(self.image_callback)
            self.executor.add_node(self.node)
            self.executor.wake()

    def image_callback(self, rgb, depth):
        rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb, "rgb8")
        self.new_rgb_image.emit(rgb_img) 

        depth_img = self.cv_bridge.imgmsg_to_cv2(depth, "32FC1") # check encoding
        self.new_depth_image.emit(depth_img)

class MainWindow(QMainWindow):
    def __init__(self, env):
        super().__init__()

        # initialize the GUI
        self.initUI()

        # start ROS image subscriber
        self.image_subscriber = None        
        self.image_topic_name=""
        self.depth_topic_name=""

        # environment for recording data
        self.env = env
        self.threadpool = QThreadPool()

        # GUI application parameters
        self.mode = "pick"
        self.table_height = 0.0
        self.x = 0.0
        self.y = 0.0
        self.gripper_rot_z = 0.0
        self.camera_intrinsics = None
        self.camera_extrinsics = None   

    def __del__(self):
        self.image_subscriber.exit()

    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_screen = QVBoxLayout(central_widget)
        horizontal_panes = QHBoxLayout()
        left_pane = QVBoxLayout()
        left_pane.setAlignment(Qt.AlignmentFlag.AlignTop)
        left_pane.setSpacing(2)
        right_pane = QVBoxLayout()
        right_pane.setAlignment(Qt.AlignmentFlag.AlignTop)
        right_pane.setSpacing(2)

        # Left Pane
        self.label_image = QLabel()
        self.label_image.mousePressEvent = self.update_pixel_coordinates
        left_pane.addWidget(self.label_image)
        horizontal_panes.addLayout(left_pane)

        # config upload
        self.upload_label = QLabel("Upload Application Config:")
        self.upload_button = QPushButton("Upload Application Parameters")
        self.upload_button.setFixedHeight(20)
        self.upload_button.clicked.connect(self.upload_camera_params)

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

        # button to update application parameters
        self.submit_button = QPushButton("Submit Application Parameters")
        self.submit_button.clicked.connect(self.update_application_params)
        self.submit_button.setFixedHeight(20)
        
        # step environment button
        self.environment_label = QLabel("Interact with Environment:")

        # reset environment button
        self.reset_button = QPushButton("Reset Environment")
        self.reset_button.clicked.connect(self.env_reset)
        self.reset_button.setFixedHeight(20)

        # step environment button
        self.step_button = QPushButton("Step Environment")
        self.step_button.clicked.connect(self.env_step)
        self.step_button.setFixedHeight(20)
    
        # episode done button
        self.done_button = QPushButton("Done Environment")
        self.done_button.clicked.connect(self.env_done)
        self.done_button.setFixedHeight(20)

        # stop data collection
        self.stop_button = QPushButton("Stop Data Collection")
        self.stop_button.clicked.connect(self.stop_data_collection)
        self.stop_button.setFixedHeight(20)

        # add all widgets to right pane
        right_pane.addWidget(self.upload_label)
        right_pane.addWidget(self.upload_button)
        right_pane.addWidget(self.label_pixel_coords)
        right_pane.addLayout(pixel_coords_box)
        right_pane.addWidget(self.label_gripper_orientation)
        right_pane.addWidget(self.gripper_rot_z_edit)
        right_pane.addWidget(self.submit_button)
        right_pane.addWidget(self.environment_label)        
        right_pane.addWidget(self.reset_button)
        right_pane.addWidget(self.step_button)
        right_pane.addWidget(self.done_button)
        right_pane.addWidget(self.stop_button)

        # add to main screen
        horizontal_panes.addLayout(right_pane)
        main_screen.addLayout(horizontal_panes)
        central_widget.setLayout(main_screen)

        self.setWindowTitle('Transporter Network Data Collection')
        self.show()

    def update_rgb_topic(self, text=None):
        if text is None:
            self.image_topic_name = self.rgb_topic_name.text()
        else:
            self.image_topic_name = text

        if self.image_subscriber is not None:
            self.image_subscriber.update_rgb_topic(self.image_topic_name)
        elif self.depth_topic_name!="":
            self.image_subscriber = ImageSubscriber(self.image_topic_name, self.depth_topic_name)
            self.image_subscriber.new_rgb_image.connect(self.update_image)
            self.image_subscriber.new_depth_image.connect(self.update_depth_image)
            self.image_subscriber.start()
        else:
            print("both topics need to be set")

    def update_depth_topic(self, text=None):
        if text is None:
            self.depth_topic_name = self.depth_topic_name.text()
        else:
            self.depth_topic_name = text

        if self.image_subscriber is not None:
            self.image_subscriber.update_depth_topic(self.depth_topic_name)
        elif self.image_topic_name!="":
            self.image_subscriber = ImageSubscriber(self.image_topic_name, self.depth_topic_name)
            self.image_subscriber.new_rgb_image.connect(self.update_image)
            self.image_subscriber.new_depth_image.connect(self.update_depth_image)
            self.image_subscriber.start()
        else:
            print("both topics need to be set")

    def update_pixel_coordinates(self, event: QMouseEvent):
        point = event.pos()
        x = point.x()
        y = point.y()
        self.x_edit.setText(str(x))
        self.y_edit.setText(str(y))

    def upload_camera_params(self):
        fileName, _ = QFileDialog.getOpenFileName(self, "Open Application Parameters File", "", "YAML Files (*.yaml)")
        if fileName:
            with open(fileName, "r") as file:
                self.config = yaml.load(file, Loader=yaml.FullLoader)
                
            # expose config to env for dataset metadata
            self.env.set_metadata(self.config)

            # table height
            self.table_height = self.config["workspace"]["table_height_offset"]

            # camera topic
            if self.config["camera"]["image_topic"]!="":
                self.update_rgb_topic(self.config["camera"]["image_topic"])

            if self.config["camera"]["depth_topic"]!="":
                self.update_depth_topic(self.config["camera"]["depth_topic"])
            
            # assign camera intrinsics
            fx = self.config["camera"]["intrinsics"]["fx"]
            fy = self.config["camera"]["intrinsics"]["fy"]
            cx = self.config["camera"]["intrinsics"]["cx"]
            cy = self.config["camera"]["intrinsics"]["cy"]
            self.camera_intrinsics = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
            
            # assign camera extrinsics
            translation = [
                self.config["camera"]["extrinsics"]["x"],
                self.config["camera"]["extrinsics"]["y"],
                self.config["camera"]["extrinsics"]["z"],
                ]
            quaternion = [
                self.config["camera"]["extrinsics"]["qx"],
                self.config["camera"]["extrinsics"]["qy"],
                self.config["camera"]["extrinsics"]["qz"],
                self.config["camera"]["extrinsics"]["qw"],
                ]
            rotation = st.Rotation.from_quat(quaternion).as_matrix()
            self.camera_extrinsics = np.eye(4)
            self.camera_extrinsics[:3, :3] = rotation
            self.camera_extrinsics[:3, 3] = translation
            

    def update_application_params(self):
        self.x = int(self.x_edit.text())
        self.y = int(self.y_edit.text())
        self.gripper_rot_z =  float(self.gripper_rot_z_edit.text()) 

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
        self.rgb_image = rgb_img.copy() 

        # overlay a line centred at pixel coord with gripper orientation
        x = int(self.x)
        y = int(self.y)
        gripper_rot_z = self.gripper_rot_z
        self.overlay_gripper_line(rgb_img, (x, y), 50, gripper_rot_z)

        # display the image
        height, width, channel = rgb_img.shape
        qimg = QImage(rgb_img.data, width, height, QImage.Format(13))
        pixmap = QPixmap.fromImage(qimg)
        self.label_image.setPixmap(pixmap)

    def update_depth_image(self, depth_img):        
        self.depth_image = depth_img.copy() 

    def env_step(self):
        depth_img = self.depth_image.copy()
        u = self.x
        v = self.y
        
        # start by inpainting depth values (sometimes sensor returns nan/inf)
        nan_mask = np.isnan(depth_img)
        inf_mask = np.isinf(depth_img)
        mask = np.logical_or(nan_mask, inf_mask)
        mask = cv2.UMat(mask.astype(np.uint8))
        scale = np.ma.masked_invalid(np.abs(depth_img)).max() # scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        depth_img = depth_img.astype(np.float32) / scale  # Has to be float32, 64 not supported.
        depth_img = cv2.inpaint(depth_img, mask, 1, cv2.INPAINT_NS)

        # interpolate remaining nan values with nearest neighbor
        depth_img = np.array(depth_img.get())
        y, x = np.where(~np.isnan(depth_img))
        x_range, y_range = np.meshgrid(np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
        depth_img = griddata((x, y), depth_img[y, x], (x_range, y_range), method='nearest')
        depth_img = depth_img * scale 
        depth_val = depth_img[v, u]

        # convert current pixels coordinates to camera frame coordinates
        pixel_coords = np.array([u, v])
        image_coords = np.concatenate([pixel_coords, np.ones(1)])
        camera_coords =  np.linalg.inv(self.camera_intrinsics) @ image_coords
        camera_coords *= depth_val # negate depth when using mujoco camera convention

        # convert camera coordinates to world coordinates
        camera_coords = np.concatenate([camera_coords, np.ones(1)])
        world_coords = self.camera_extrinsics @ camera_coords
        world_coords = world_coords[:3] / world_coords[3]

        print("World Coordinates:", world_coords)
        
        quat = R.from_euler('xyz', [0, 180, -self.gripper_rot_z+180], degrees=True).as_quat()
        pose = np.concatenate([world_coords, quat])

        action_dict = {
            "pose": pose, 
            "pixel_coords": np.array([u, v]),
            "gripper_rot": -self.gripper_rot_z + 180, # defined wrt base frame, note z-axis of gripper frame points in direction of grasp
        }

        if self.mode == "pick":
            self.env.set_observation(self.rgb_image, self.depth_image)
            worker = Worker(self.env.step, action_dict)
        else:
            self.env.set_observation(self.rgb_image, self.depth_image)
            worker = Worker(self.env.step, action_dict)
        
        self.threadpool.start(worker)
    
    def env_reset(self):
        self.env.reset()
        self.env.set_observation(self.rgb_image, self.depth_image)

    def env_done(self):
        self.env.set_observation(self.rgb_image, self.depth_image)
        self.env.done_step()

    def stop_data_collection(self):
        self.env.close()
        self.close()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_ip", default="192.168.106.99", required=False)
    parser.add_argument("--use_fake_hardware", default="false", required=False)
    parser.add_argument("--use_gripper", default="true", required=False)
    parser.add_argument("--gripper_controller", default="/robotiq/robotiq_gripper_controller/gripper_cmd", required=False)
    args = parser.parse_args()

    # TFDS dataset configuration
    dataset_config = tfds.rlds.rlds_base.DatasetConfig(
        name="transport_cubes",
        observation_info=tfds.features.FeaturesDict({
            "overhead_camera/rgb": tfds.features.Tensor(shape=(621,1104, 3), dtype=np.uint8),
            "overhead_camera/depth": tfds.features.Tensor(shape=(621,1104), dtype=np.float32),
        }),
        action_info=tfds.features.FeaturesDict({
            "pose": tfds.features.Tensor(shape=(7,), dtype=np.float64),
            "pixel_coords": tfds.features.Tensor(shape=(2,), dtype=np.int64),
            "gripper_rot": np.float64,
        }),
        reward_info=tf.float64,
        discount_info=tf.float64,
        episode_metadata_info={
            "image_topic": tf.string,
            "depth_topic": tf.string,
            "intrinsics":{
                "fx": tf.float64,
                "fy": tf.float64,
                "cx": tf.float64,
                "cy": tf.float64,  
            },
            "extrinsics":{
                "x": tf.float64,
                "y": tf.float64,
                "z": tf.float64,
                "qw": tf.float64,   
                "qx": tf.float64,
                "qy": tf.float64,
                "qz": tf.float64,
            },
        },
        )
    
    def calibration_metadata(timestep, unused_action, unused_env):
        """
        Store camera calibration params as episode metadata.
        """
        if timestep.first:
            return unused_env.metadata["camera"]
        else:
            return None

    rclpy.init(args=None)
    env = FrankaTable(args)
    os.makedirs(os.path.join(os.path.dirname(__file__), "data"), exist_ok=True)
    with envlogger.EnvLogger(
            env, 
            episode_fn=calibration_metadata,
            backend = tfds_backend_writer.TFDSBackendWriter(
                data_directory=os.path.join(os.path.dirname(__file__), "data"),
                split_name='train',
                max_episodes_per_file=1,
                ds_config=dataset_config,
            ),
            ) as env:
        app = QApplication(sys.argv)
        ex = MainWindow(env)
        app.exec()

if __name__ == '__main__':
    main()

