import sys
import math
import numpy as np
import scipy.spatial.transform as st
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QFileDialog
from PyQt6.QtGui import QPixmap, QImage, QMouseEvent
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2

from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        logger.info("Trajectory: {}".format(robot_trajectory))
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


class ImageSubscriber(QThread):
    new_image = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.cv_bridge = CvBridge()

    def run(self):
        rclpy.init(args=None)
        node = rclpy.create_node('image_subscriber')
        subscription = node.create_subscription(Image, 'overhead_camera', self.image_callback, 10)
        rclpy.spin(node)

    def image_callback(self, msg):
        rgb_img = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        self.new_image.emit(rgb_img)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # initialize the GUI
        self.initUI()

        # start ROS image subscriber
        self.image_subscriber = ImageSubscriber()
        self.image_subscriber.new_image.connect(self.update_image)
        self.image_subscriber.start()

        # start moveit python interface
        robot_ip = "" # not applicable for fake hardware
        use_gripper = "true" 
        use_fake_hardware = "true" 

        moveit_config = (
            MoveItConfigsBuilder(robot_name="panda", package_name="franka_robotiq_moveit_config")
            .robot_description(file_path=get_package_share_directory("franka_robotiq_description") + "/urdf/robot.urdf.xacro",
                mappings={
                    "robot_ip": robot_ip,
                    "robotiq_gripper": use_gripper,
                    "use_fake_hardware": use_fake_hardware,
                    })
            .robot_description_semantic("config/panda.srdf.xacro", 
                mappings={
                    "robotiq_gripper": use_gripper,
                    })
            .trajectory_execution("config/moveit_controllers.yaml")
            .moveit_cpp(
                file_path=get_package_share_directory("panda_motion_planning_demos")
                + "/config/moveit_cpp.yaml"
            )
            .to_moveit_configs()
            ).to_dict()

        self.panda = MoveItPy(config_dict=moveit_config)
        self.panda_arm = self.panda.get_planning_component("panda_arm")        

        # application parameters
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
        self.label_image.mousePressEvent = self.updateCoordinates  # Connect mousePressEvent to getImageCoordinate function
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
        self.line_edit.returnPressed.connect(self.updateParams)  # Connect returnPressed signal to updateTableHeight function

        # camera calibration parameters
        self.label_camera_params = QLabel("Camera Calibration Parameters:")
        self.label_camera_params.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.label_camera_params.setFixedHeight(20)
        
        # button to upload file containing camera calibration parameters
        self.upload_button = QPushButton("Upload Camera Parameters")
        self.upload_button.setFixedHeight(20)
        self.upload_button.clicked.connect(self.uploadCameraParams)  # Connect clicked signal to uploadCameraParams function

        # outline selected coordinates

        # pixel coordinates
        self.label_pixel_coords = QLabel("Selected Pixel Coordinates:")
        self.label_pixel_coords.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.label_pixel_coords.setFixedHeight(20)
            
        pixel_coords_box = QHBoxLayout()
        
        # X pixel coordinate input
        self.x_edit = QLineEdit()
        self.x_edit.setPlaceholderText("X")
        self.x_edit.returnPressed.connect(self.updateCoordinates)
        self.x_edit.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.x_edit.setFixedHeight(20)

        # Y pixel coordinate input
        self.y_edit = QLineEdit()
        self.y_edit.setPlaceholderText("Y")
        self.y_edit.returnPressed.connect(self.updateCoordinates)
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
        self.submit_button = QPushButton("Submit Settings")
        self.submit_button.clicked.connect(self.updateParams)  # Connect clicked signal to updateTableHeight function
        self.submit_button.setFixedHeight(20)

        # motion planning button
        self.execute_button = QPushButton("Execute Motion Plan")
        self.execute_button.clicked.connect(self.executeMotionPlan)  # Connect clicked signal to executeMotionPlan function
        self.execute_button.setFixedHeight(20)

        right_pane.addWidget(self.label_table_height)
        right_pane.addWidget(self.line_edit)
        right_pane.addWidget(self.label_pixel_coords)
        right_pane.addLayout(pixel_coords_box)
        right_pane.addWidget(self.label_gripper_orientation)
        right_pane.addWidget(self.gripper_rot_z_edit)
        right_pane.addWidget(self.label_camera_params)
        right_pane.addWidget(self.upload_button)
        right_pane.addWidget(self.submit_button)
        right_pane.addWidget(self.execute_button)

        horizontal_panes.addLayout(right_pane)

        main_screen.addLayout(horizontal_panes)
        central_widget.setLayout(main_screen)

        self.setWindowTitle('Transporter Network Data Collection')
        self.show()

    def updateCoordinates(self, event: QMouseEvent):
        point = event.pos()
        x = point.x()
        y = point.y()
        self.x_edit.setText(str(x))
        self.y_edit.setText(str(y))

    def uploadCameraParams(self):
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


    def updateParams(self):                                                                                                         # Code to update table height parameter
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
    

    def pick(self, world_coords):
        pick_pose_msg = PoseStamped()
        pick_pose_msg.header.frame_id = "panda_link0"
        pick_pose_msg.pose.orientation.x = 0.9238795
        pick_pose_msg.pose.orientation.y = -0.3826834
        pick_pose_msg.pose.orientation.z = 0.0
        pick_pose_msg.pose.orientation.w = 0.0
        pick_pose_msg.pose.position.x = world_coords[0]
        pick_pose_msg.pose.position.y = world_coords[1]
        pick_pose_msg.pose.position.z = world_coords[2]
        
        # prepick pose
        panda_arm.set_start_state_to_current_state()
        pre_pick_pose_msg = deepcopy(pick_pose_msg)
        pre_pick_pose_msg.pose.position.z += 0.1
        panda_arm.set_goal_state(pose_stamped_msg=pre_pick_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        # pick pose
        panda_arm.set_start_state_to_current_state()
        panda_arm.set_goal_state(pose_stamped_msg=pick_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        # close gripper
        gripper_client.close_gripper()
        time.sleep(2.0)
        
        # raise arm
        panda_arm.set_start_state_to_current_state()
        pre_pick_pose_msg.pose.position.z += 0.2
        panda_arm.set_goal_state(pose_stamped_msg=pre_pick_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        self.mode = "place"

    def place(self, world_coords):
        place_pose_msg = PoseStamped()
        place_pose_msg.header.frame_id = "panda_link0"
        place_pose_msg.pose.orientation.x = 0.9238795
        place_pose_msg.pose.orientation.y = -0.3826834
        place_pose_msg.pose.orientation.z = 0.0
        place_pose_msg.pose.orientation.w = 0.0
        place_pose_msg.pose.position.x = world_coords[0]
        place_pose_msg.pose.position.y = world_coords[1]
        place_pose_msg.pose.position.z = world_coords[2]
        
        # preplace pose
        panda_arm.set_start_state_to_current_state()
        pre_place_pose_msg = deepcopy(place_pose_msg)
        pre_place_pose_msg.pose.position.z += 0.1
        panda_arm.set_goal_state(pose_stamped_msg=pre_place_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        # place pose
        panda_arm.set_start_state_to_current_state()
        panda_arm.set_goal_state(pose_stamped_msg=place_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        # open gripper
        gripper_client.open_gripper()
        time.sleep(2.0)
        
        # raise arm
        panda_arm.set_start_state_to_current_state()
        pre_place_pose_msg.pose.position.z += 0.2
        panda_arm.set_goal_state(pose_stamped_msg=pre_place_pose_msg, pose_link="panda_link8")
        plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)

        self.mode = "pick"
    
    def executeMotionPlan(self):
        
        # map pixel coords to world coords
        
        ## get depth value from pixel coordinates
        # TODO: get depth values from camera depth image
        depth_val = 0.0

        ## convert pixels to camera frame coordinates
        pixel_coords = np.array([self.x, self.y])
        image_coords = np.concatenate([pixel_coords, np.ones(1)])
        camera_coords =  np.linalg.inv(self.camera_intrinsics) @ image_coords
        camera_coords *= -depth_val # negative sign due to mujoco camera convention

        ## convert camera coordinates to world coordinates
        camera_coords = np.concatenate([camera_coords, np.ones(1)])
        world_coords = np.linalg.inv(self.camera_extrinsics) @ camera_coords
        world_coords = world_coords[:3] / world_coords[3]

        if self.mode == "pick":
            self.pick(world_coords)
        else:
            self.place(world_coords)


def main(args=None):
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()

