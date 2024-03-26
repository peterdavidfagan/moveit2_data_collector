import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QWidget
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import QTimer

# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

        # Simulate a ROS topic update every 1000 ms (1 second)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateImage)
        self.timer.start(1000)  # milliseconds

    def initUI(self):
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        vbox = QVBoxLayout(central_widget)

        # Create QHBoxLayout for image, numerical input, and buttons
        hbox_top = QHBoxLayout()

        # Create QLabel for image
        self.label_image = QLabel()
        hbox_top.addWidget(self.label_image)

        # Create QHBoxLayout for numerical input
        hbox_numerical = QHBoxLayout()

        # Create QLabel for title
        label_title = QLabel("Table Height")
        hbox_numerical.addWidget(label_title)

        # Create QLineEdit for numerical input
        self.line_edit = QLineEdit()
        self.line_edit.returnPressed.connect(self.updateTableHeight)  # Connect returnPressed signal to updateTableHeight function
        hbox_numerical.addWidget(self.line_edit)

        # Create QPushButton for submitting value
        submit_button = QPushButton("Submit")
        submit_button.clicked.connect(self.updateTableHeight)  # Connect clicked signal to updateTableHeight function
        hbox_numerical.addWidget(submit_button)

        hbox_top.addLayout(hbox_numerical)

        vbox.addLayout(hbox_top)

        # Create QPushButton for executing motion plan
        execute_button = QPushButton("Execute Motion Plan")
        execute_button.clicked.connect(self.executeMotionPlan)  # Connect clicked signal to executeMotionPlan function
        vbox.addWidget(execute_button)

        central_widget.setLayout(vbox)

        self.setWindowTitle('Image and Numerical Input')
        self.show()

    def updateImage(self):
        # Code to update image from ROS topic
        # For demonstration, updating image with a dummy image
        pixmap = QPixmap("image.jpg")  # Provide path to your image
        self.label_image.setPixmap(pixmap)

    def executeMotionPlan(self):
        # Code to execute motion planning function
        print("Executing Motion Plan")

    def updateTableHeight(self):
        # Code to update table height parameter
        table_height = float(self.line_edit.text())  # Convert input text to float
        print("Updating Table Height:", table_height)

def main():
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()


