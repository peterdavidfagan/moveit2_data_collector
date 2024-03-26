import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QWidget
from PyQt6.QtGui import QPixmap, QImage, QMouseEvent
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2

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

        # Initialize the GUI
        self.initUI()

        # Start ROS image subscriber
        self.image_subscriber = ImageSubscriber()
        self.image_subscriber.new_image.connect(self.update_image)
        self.image_subscriber.start()

    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_screen = QVBoxLayout(central_widget)
        horizontal_panes = QHBoxLayout()
        left_pane = QVBoxLayout()
        right_pane = QVBoxLayout()
        right_pane.setSpacing(0)

        # Left Pane
        self.label_image = QLabel()
        self.label_image.mousePressEvent = self.updateCoordinates  # Connect mousePressEvent to getImageCoordinate function
        left_pane.addWidget(self.label_image)
        horizontal_panes.addLayout(left_pane)


        # Right Pane

        # table height input
        self.label_table_height = QLabel("Enter Table Height:")
        self.label_table_height.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.label_table_height.setFixedHeight(20)
        self.line_edit = QLineEdit()
        self.line_edit.setPlaceholderText("Enter table height")
        self.line_edit.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.line_edit.setFixedHeight(20)
        self.line_edit.returnPressed.connect(self.updateParams)  # Connect returnPressed signal to updateTableHeight function
        self.submit_button = QPushButton("Submit Settings")
        self.submit_button.clicked.connect(self.updateParams)  # Connect clicked signal to updateTableHeight function
        self.submit_button.setFixedHeight(20)

        # outline selected coordinates
        # X Coordinate Input
        self.x_edit = QLineEdit()
        self.x_edit.setPlaceholderText("X")
        self.x_edit.returnPressed.connect(self.updateCoordinates)
        self.x_edit.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.x_edit.setFixedHeight(20)

        # Y Coordinate Input
        self.y_edit = QLineEdit()
        self.y_edit.setPlaceholderText("Y")
        self.y_edit.returnPressed.connect(self.updateCoordinates)
        self.y_edit.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.y_edit.setFixedHeight(20)

        # motion planning button
        self.execute_button = QPushButton("Execute Motion Plan")
        self.execute_button.clicked.connect(self.executeMotionPlan)  # Connect clicked signal to executeMotionPlan function
        self.execute_button.setFixedHeight(20)

        right_pane.addWidget(self.label_table_height)
        right_pane.addWidget(self.line_edit)
        right_pane.addWidget(self.x_edit)
        right_pane.addWidget(self.y_edit)
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


    def executeMotionPlan(self):                                                                                                                                                                                                                                           
        # Code to execute motion planning function                                                                                                                                                                                                                         
        print("Executing Motion Plan")                                                                                                                                                                                                                                     
                                                                                                                                                                                                                                                                           
    def updateParams(self):                                                                                                                                                                                                                                                
        # Code to update table height parameter                                                                                                                                                                                                                            
        self.table_height = float(self.line_edit.text())  # Convert input text to float                                                                                                                                                                                    
        self.x = float(self.x_edit.text())                                                                                                                                                                                                                                 
        self.y = float(self.y_edit.text())                                                                                                                                                                                                                                 
        print("Updating Table Height:", self.table_height)                                                                                                                                                                                                                 
        print("Updating X Coordinate:", self.x)                                                                                                                                                                                                                            
        print("Updating Y Coordinate:", self.y)     


    def update_image(self, rgb_img):
        height, width, channel = rgb_img.shape
        bytes_per_line = channel * width
        qimg = QImage(rgb_img.data, width, height, QImage.Format(13))
        pixmap = QPixmap.fromImage(qimg)
        self.label_image.setPixmap(pixmap)

def main(args=None):
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()

