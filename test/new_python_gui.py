#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QMessageBox, QPushButton, QSizePolicy, QToolBar, QScrollArea, QButtonGroup, QRadioButton, QAction, QGroupBox, QLabel, QLineEdit, QFormLayout, QDialog, QInputDialog, QMenuBar, QMenu, QComboBox, QCheckBox
from PyQt5 import QtCore

#--------------------------------------------------------------------------- MOVE TO FRAME ACTIONBLOCK ---------------------------------------------------------------------------#
class MoveToFrameParameterizer(QDialog):
    def __init__(self, parent=None):
        super(MoveToFrameParameterizer, self).__init__(parent)
        self.setWindowTitle("Parameterize MoveToFrame-Actionblock")

        # Create input fields for parameters
        self.move_group_combo = QComboBox()
        self.move_group_combo.addItems(["Cam1", "Laser", "Tool"])
        self.target_frame_combo = QComboBox()
        self.target_frame_combo.addItems(["Camera_Station_TCP", "Tool_TCP", "Laser"])
        self.name_edit = QLineEdit()
        self.offset_edit = QLineEdit()
        self.offset_edit.setText("0.0, 0.0, 0.0")
        self.wait_input_checkbox = QCheckBox("Wait for User Input")
        form_layout = QFormLayout()
        form_layout.addRow("Name:", self.name_edit)
        form_layout.addRow("MoveGroup:", self.move_group_combo)
        form_layout.addRow("TargetFrame:", self.target_frame_combo)
        form_layout.addRow("Offset (x, y, z):", self.offset_edit)         # Translation = Offset is required for safety reasons
        form_layout.addRow(self.wait_input_checkbox)

        # Create save- and back-buttons
        self.save_button = QPushButton("Save")
        self.cancel_button = QPushButton("Cancel")
        self.save_button.clicked.connect(self.save_parameters)
        self.cancel_button.clicked.connect(self.close)
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.save_button)
        button_layout.addWidget(self.cancel_button)

        # general layout
        layout = QVBoxLayout()
        layout.addLayout(form_layout)
        layout.addLayout(button_layout)
        self.setLayout(layout)

        # create empty dictionary for parameters
        self.saved_parameters = {}

    def save_parameters(self):
        # Call accept() to close the dialog with an "Accepted" result code
        self.saved_parameters = self.get_parameters()
        self.accept()

    def get_parameters(self):
        # return self.parameters
        parameters = {
            "name": self.name_edit.text(),
            "MoveGroup": self.move_group_combo.currentText(),
            "TargetFrame": self.target_frame_combo.currentText(),
            "Offset (x, y, z)": self.offset_edit.text(),
        }

        parameters["Wait for User Input"] = self.wait_input_checkbox.isChecked()

        return parameters
#--------------------------------------------------------------------------- MOVE ABSOLUTE ACTIONBLOCK---------------------------------------------------------------------------#
class MoveAbsoluteParameterizer(QDialog):
    def __init__(self, parent=None):
        super(MoveAbsoluteParameterizer, self).__init__(parent)
        self.setWindowTitle("Parameterize Move absolute-Actionblock")

        self.name_edit = QLineEdit()
        self.position_edit = QLineEdit()
        self.position_edit.setText("0.0, 0.0, 0.0")
        self.orientation_edit = QLineEdit()
        self.orientation_edit.setText("0.0, 0.0, 0.0, 1.0")
        self.wait_input_checkbox = QCheckBox("Wait for User Input")

        form_layout = QFormLayout()
        form_layout.addRow("Name:", self.name_edit)
        form_layout.addRow("Position (x, y, z):", self.position_edit)
        form_layout.addRow("Orientation (x, y, z, w):", self.orientation_edit)

        # Create save- and back-buttons
        self.save_button = QPushButton("Save")
        self.cancel_button = QPushButton("Cancel")
        self.save_button.clicked.connect(self.save_parameters)
        self.cancel_button.clicked.connect(self.close)
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.save_button)
        button_layout.addWidget(self.cancel_button)

        # general layout
        layout = QVBoxLayout()
        layout.addLayout(form_layout)
        layout.addLayout(button_layout)
        self.setLayout(layout)

        # create empty dictionary for parameters
        self.saved_parameters = {}
       
    def save_parameters(self):
        self.saved_parameters = self.get_parameters()
        self.accept()

    def get_parameters(self):
        # return self.parameters
        parameters = {
            "name": self.name_edit.text(),
            "Position (x, y, z)": self.position_edit.text(),
            "Orientation (x, y, z, w)": self.orientation_edit.text(),
        }

        parameters["Wait for User Input"] = self.wait_input_checkbox.isChecked()

        return parameters

#--------------------------------------------------------------------------- PICK ACTIONBLOCK ---------------------------------------------------------------------------#
class PickParameterizer(QDialog):
    def __init__(self, parent=None):
        super(PickParameterizer, self).__init__(parent)
        self.setWindowTitle("Parameterize Pick-Actionblock")
            
    def save_parameters(self):
        self.saved_parameters = self.get_parameters()
        self.accept()

    def get_parameters(self):
        parameters = {
            "name": self.name_edit.text(),
        }

        parameters["Wait for User Input"] = self.wait_input_checkbox.isChecked()

        return parameters


#--------------------------------------------------------------------------- PLACE ACTIONBLOCK ---------------------------------------------------------------------------#
class PlaceParameterizer(QDialog):
    def __init__(self, parent=None):
        super(PlaceParameterizer, self).__init__(parent)
        self.setWindowTitle("Parameterize Place-Actionblock")
            
    def save_parameters(self):
        self.saved_parameters = self.get_parameters()
        self.accept()

    def get_parameters(self):
        parameters = {
            "name": self.name_edit.text(),
        }

        parameters["Wait for User Input"] = self.wait_input_checkbox.isChecked()

        return parameters

#--------------------------------------------------------------------------- APPLY ADHESIVE ACTIONBLOCK---------------------------------------------------------------------------#
class ApplyAdhesiveParameterizer(QDialog):
    def __init__(self, parent=None):
        super(ApplyAdhesiveParameterizer, self).__init__(parent)
        self.setWindowTitle("Parameterize ApplyAdhesive-Actionblock")
           
    def save_parameters(self):
        self.saved_parameters = self.get_parameters()
        self.accept()

    def get_parameters(self):
        parameters = {
            "name": self.name_edit.text(),
        }

        parameters["Wait for User Input"] = self.wait_input_checkbox.isChecked()

        return parameters
#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
#--------------------------------------------------------------------------- FRAMEWORK MAIN WINDOW ---------------------------------------------------------------------------#
class PMProgrammingFramework(QMainWindow):
    def __init__(self):
        super(PMProgrammingFramework, self).__init__()
        self.setWindowTitle("PM_programming_framework")

        # Set the minimum size of the window
        self.setMinimumSize(1200, 800)

        # Create the main widget and layout
        self.main_widget = QWidget()
        self.layout = QHBoxLayout(self.main_widget)

        # ---------------------------- TOOL BAR ----------------------------
        # Add buttons for parameterizable ActionBlocks to the toolbar
        self.action_buttons = {
            "MoveToFrame": QPushButton("MoveToFrame"),
            "Move absolute": QPushButton("Move absolute"),
            "Pick": QPushButton("Pick"),
            "Place": QPushButton("Place"),
            "Apply adhesive": QPushButton("Apply adhesive"),
        }

        # Vertical layout for action block buttons
        action_buttons_layout = QVBoxLayout()
        for action, button in self.action_buttons.items():
            button.clicked.connect(lambda checked, action=action: self.open_parameterizer(action))
            action_buttons_layout.addWidget(button)

        self.layout.addLayout(action_buttons_layout)

        # ---------------------------- PROGRAM AREA ----------------------------
            
        self.program_area = QGroupBox("Program Area")
        self.program_layout = QVBoxLayout()
        self.program_area.setLayout(self.program_layout)  # Set the layout to program_area

        self.program_area.setStyleSheet("background-color: white;")
  
        # Create a QWidget container to hold all the saved ActionBlocks
        self.saved_action_blocks_container = QWidget()
        self.saved_action_blocks_layout = QVBoxLayout(self.saved_action_blocks_container)
        self.program_layout.addWidget(self.saved_action_blocks_container)

        # Use QScrollArea for the program_area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(self.program_area)
        self.layout.addWidget(scroll_area)

        self.setCentralWidget(self.main_widget)

        # ---------------------------- MENU BAR ----------------------------
        # Create the menu bar

        self.top_toolbar = QToolBar("Menu")
        self.addToolBar(self.top_toolbar)

        # Add "Save Program" button to the top toolbar
        self.save_program_button = QPushButton("Save Program")
        self.save_program_button.clicked.connect(self.save_program)
        self.top_toolbar.addWidget(self.save_program_button)

        # Add "Clear Program" button to the top toolbar
        self.clear_program_button = QPushButton("Clear Program")
        self.clear_program_button.clicked.connect(self.clear_program)
        self.top_toolbar.addWidget(self.clear_program_button)

        # Add "Execute Program" button to the top toolbar
        self.execute_program_button = QPushButton("Execute Program")
        self.execute_program_button.clicked.connect(self.execute_program)
        self.top_toolbar.addWidget(self.execute_program_button)

        # Add space between "Clear Program" button and toggle button
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.top_toolbar.addWidget(spacer)

        # Add the toggle button for "Simulation Mode" and "Real Mode" to the top menu
        self.mode_toggle_group = QButtonGroup(self)
        self.simulation_mode_button = QRadioButton("Simulation Mode")
        self.real_mode_button = QRadioButton("Real Mode")
        self.mode_toggle_group.addButton(self.simulation_mode_button)
        self.mode_toggle_group.addButton(self.real_mode_button)
        self.top_toolbar.addWidget(self.simulation_mode_button)
        self.top_toolbar.addWidget(self.real_mode_button)

        # Connect the mode_toggle_group to a function to handle mode changes
        self.mode_toggle_group.buttonClicked.connect(self.handle_mode_change)

        # Set "Simulation Mode" as the default mode
        self.simulation_mode_button.setChecked(True)
        self.handle_mode_change(show_info=False)  # No need to show info for default mode

        # Create a list to save the saved ActionBlocks
        self.saved_action_blocks = []

        # Update the layout for the program area
        self.program_layout = QVBoxLayout(self.program_area)
        self.program_layout.setAlignment(QtCore.Qt.AlignTop)
        self.program_area.setLayout(self.program_layout)

    def handle_mode_change(self, show_info=True):
        # launch sim or hw file, dont know how to handle it in the GUI...
        mode = "Simulation Mode" if self.simulation_mode_button.isChecked() else "Real Mode"
        if mode == "Real Mode":
            # Add functionality for "Real Mode"
            print("Real Mode is selected.")
        else:
            # Add functionality for "Simulation Mode"
            print("Simulation Mode is selected.")

        if show_info:
            QMessageBox.information(self, "Mode Change", f"Switched to {mode}")

    def open_parameterizer(self, action_class):
        if action_class == "MoveToFrame":
            parameterizer = MoveToFrameParameterizer(self)
        elif action_class == "Move absolute":
            parameterizer = MoveAbsoluteParameterizer(self)
        elif action_class == "Pick":
            parameterizer = PickParameterizer(self)
        elif action_class == "Place":
            parameterizer = PlaceParameterizer(self)
        elif action_class == "Apply adhesive":
            parameterizer = ApplyAdhesiveParameterizer(self)
        # else:
        #     # Create parameterizer for other ActionBlocks as needed
        #     parameterizer = QDialog(self)
        #     parameterizer.setWindowTitle(f"Parameterize {action_class}")

        result = parameterizer.exec_()
        if result == QDialog.Accepted:
            parameters = parameterizer.get_parameters()
            if parameters:
                # Add the parameterized action block to the program area
                action_block_class = action_class
                action_block_name = parameters["name"]
                self.saved_action_blocks.append((action_block_class, action_block_name))

                # Update the program area with the saved ActionBlocks
                self.update_program_area()

    def update_program_area(self):
    # Clear the existing layout
        for i in reversed(range(self.saved_action_blocks_layout.count())):
            self.saved_action_blocks_layout.itemAt(i).widget().setParent(None)

        # Add the saved ActionBlocks to the program layout
        for action_class, action_name in self.saved_action_blocks:
            parameters = self.get_action_block_parameters(action_class, action_name)

            if parameters:
                # Create a string representation of the ActionBlock and its parameters
                parameter_str = ", ".join(f"{key}: {value}" for key, value in parameters.items() if key != "name")
                action_label = QLabel(f"{action_class} - {action_name} - {parameter_str}")
            else:
                # If no parameters, display without the parameter part
                action_label = QLabel(f"{action_class} - {action_name}")

            self.saved_action_blocks_layout.addWidget(action_label)

    def get_action_block_parameters(self, action_class, action_name):
        for index, (saved_action_class, saved_action_name) in enumerate(self.saved_action_blocks):
            if saved_action_class == action_class and saved_action_name == action_name:
                parameterizer = self.get_parameterizer_instance(action_class)
                if parameterizer:
                    return parameterizer.get_parameters()

        return None

    def get_parameterizer_instance(self, action_class):
        if action_class == "MoveToFrame":
            return MoveToFrameParameterizer(self)
        elif action_class == "Move absolute":
            return MoveAbsoluteParameterizer(self)
        elif action_class == "Pick":
            return PickParameterizer(self)
        elif action_class == "Place":
            return PlaceParameterizer(self)
        elif action_class == "Apply adhesive":
            return ApplyAdhesiveParameterizer(self)

    def save_program(self):
        # save program in JSON-file
        pass

    def execute_program(self):
        # execute JSON file, call services, ...
        pass

    def clear_program(self):
        for i in reversed(range(self.saved_action_blocks_layout.count())):
            self.saved_action_blocks_layout.itemAt(i).widget().setParent(None)

        # Clear the saved action blocks and parameters
        self.saved_action_blocks = []

    def toggle_sim_real(self):
        pass

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PMProgrammingFramework()
    window.show()
    sys.exit(app.exec_())
