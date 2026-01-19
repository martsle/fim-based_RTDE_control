import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QCheckBox
from PyQt6.QtGui import QFont, QDoubleValidator
from calibration import Calibration
import csv

class CalibrationWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Calibration')
        self.setFixedSize(950, 900)  
        self.setFont(QFont("TUM Neue Helvetica", 18))  
        self.DEBUG = True

        layout = QVBoxLayout()

        # Eingabefeld für die Höhe der Bauplattform (in mm)
        self.height_label = QLabel("Height of the build platform (mm):")
        self.height_input = QLineEdit()
        self.height_input.setValidator(QDoubleValidator())  # Nur ganze Zahlen zulassen
        layout.addWidget(self.height_label)
        layout.addWidget(self.height_input)

        # Eingabefeld für die Breite der Bauplattform (in mm)
        self.width_label = QLabel("Width of the build platform (mm):")
        self.width_input = QLineEdit()
        self.width_input.setValidator(QDoubleValidator())
        layout.addWidget(self.width_label)
        layout.addWidget(self.width_input)

        # Eingabefeld für den Abstand zur Basis des Roboters (links) (in mm)
        self.dist_x_label = QLabel("Distance to the robot base (x) (mm):")
        self.dist_x_input = QLineEdit()
        self.dist_x_input.setValidator(QDoubleValidator())
        layout.addWidget(self.dist_x_label)
        layout.addWidget(self.dist_x_input)

        # Eingabefeld für den Abstand zur Basis des Roboters (unten) (in mm)
        self.dist_y_label = QLabel("Distance to the robot base (y) (mm):")
        self.dist_y_input = QLineEdit()
        self.dist_y_input.setValidator(QDoubleValidator())
        layout.addWidget(self.dist_y_label)
        layout.addWidget(self.dist_y_input)

        # Ckeckbox für die gekrümmte Fäche
        self.check_curvedSurface = QCheckBox("Curved Surface")

        # Eingabefeld für die Höhe der gekrümmten Fläche (unten) (in mm)
        self.curved_height = QLabel("Max. Height of the curved surface")
        self.curved_height_input = QLineEdit()
        self.curved_height_input.setValidator(QDoubleValidator())
        self.curved_height_input.setText("0.0")
        self.curved_height_input.setEnabled(False)
        self.check_curvedSurface.stateChanged.connect(self.toggleInputField)
        layout.addWidget(self.curved_height)
        layout.addWidget(self.curved_height_input)
        
        layout.addWidget(self.check_curvedSurface)

        # Button für die Kalibrierung
        self.calibrate_button = QPushButton("Calibration")
        self.calibrate_button.clicked.connect(self.calibrate)
        self.calibrate_button.setEnabled(False)
        layout.addWidget(self.calibrate_button)

        self.setLayout(layout)

        # Verknüpfung der Eingabefelder mit der Überprüfungsfunktion
        self.height_input.textChanged.connect(self.checkInputs)
        self.width_input.textChanged.connect(self.checkInputs)
        self.dist_x_input.textChanged.connect(self.checkInputs)
        self.dist_y_input.textChanged.connect(self.checkInputs)
        self.curved_height_input.textChanged.connect(self.checkInputs)

        if self.DEBUG:
            self.height_input.setText("450")
            self.width_input.setText("450")
            self.dist_x_input.setText("410")
            self.dist_y_input.setText("70")
            self.calibrate()


    def toggleInputField(self, state):
        self.curved_height_input.setEnabled(state == 2)

    def checkInputs(self):
        # Überprüfen, ob alle Eingabefelder ausgefüllt und numerisch sind
        if all([
            self.height_input.text(),
            self.height_input.hasAcceptableInput(),
            self.width_input.text(),
            self.width_input.hasAcceptableInput(),
            self.dist_x_input.text(),
            self.dist_x_input.hasAcceptableInput(),
            self.dist_y_input.text(),
            self.dist_y_input.hasAcceptableInput(),
            self.curved_height_input.text(),
            self.curved_height_input.hasAcceptableInput()
        ]):
            self.calibrate_button.setEnabled(True)
        else:
            self.calibrate_button.setEnabled(False)

    def calibrate(self):
        self.height_input.setDisabled(True)
        self.width_input.setDisabled(True)
        self.dist_x_input.setDisabled(True)
        self.dist_y_input.setDisabled(True)
        self.check_curvedSurface.setDisabled(True)
        self.curved_height_input.setDisabled(True)
        self.calibrate_button.setDisabled(True)
        self.calibrate_button.setText("Calibration running...")
        curved_height = 0.0 if not self.check_curvedSurface.isChecked() else float(self.curved_height_input.text())
        
        cal = Calibration(float(self.dist_y_input.text()), 
                          float(self.dist_x_input.text()), 
                          float(self.height_input.text()), 
                          float(self.width_input.text()),
                          self.check_curvedSurface.isChecked(),curved_height)
        
        #Tatsächliche Kalibrierung
        print("Calibration started")
        cal.calibrate()
        print("Calibration Finished")

        with open('./calibration/calibration_points.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(cal.calibratedPoints)

        self.close()

    

app = QApplication(sys.argv)
window = CalibrationWindow()
window.show()
sys.exit(app.exec_())
