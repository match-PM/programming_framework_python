from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QMessageBox, QPushButton, QSizePolicy, QToolBar, QScrollArea, QGroupBox, QDialog
from PyQt5 import QtCore
from PyQt5.QtCore import Qt

from move_to_frame_action_block import MoveToFrameParameterizer
from move_absolute_action_block import MoveAbsoluteParameterizer
from move_relative_action_block import MoveRelativeParameterizer
from grasp_action_block import GraspParameterizer
from release_action_block import ReleaseParameterizer

from spawn_component_client import SpawnComponentClient
from destroy_component_client import DestroyComponentClient
from grip_pos_action_client import GripPosActionClient
from place_pos_action_client import PlacePosActionClient
from home_pos_action_client import HomePosActionClient

from framework_funtions import update_program_area, execute_program, clear_program
from file_functions import  save_program, load_program, delete_program, save_programs_to_file, load_programs_from_file

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
#--------------------------------------------------------------------------- FRAMEWORK MAIN WINDOW ---------------------------------------------------------------------------#
class PMProgrammingFramework(QMainWindow):
    def __init__(self):
        super(PMProgrammingFramework, self).__init__()
        self.setWindowTitle("PM_programming_framework")

        # Set the minimum size of the window
        self.setMinimumSize(1200, 500)

        # Initialize empty dictionarys
        self.saved_action_blocks = {}
        self.saved_programs = {}
        self.programs = {}

        self.create_gui()
        load_programs_from_file(self)

    def create_gui(self):
        # Create the main widget and layout
        self.main_widget = QWidget()
        self.layout = QHBoxLayout(self.main_widget)

        # ---------------------------- TOOL BAR ----------------------------
        # Add buttons for parameterizable ActionBlocks to the toolbar
        self.action_buttons = {
            "MoveToFrame": QPushButton("Move To Frame"),
            "MoveAbsolute": QPushButton("Move Absolute"),
            "MoveRelative": QPushButton("Move Relative"),
            "Grasp": QPushButton("Gripper Grasp"),
            "Release": QPushButton("Gripper Release")
        }

        action_buttons_layout = QVBoxLayout()
        action_buttons_layout.setSpacing(10)  # Set spacing between buttons to 10
        action_buttons_layout.setAlignment(Qt.AlignTop)  # Align buttons to the top

        for action, button in self.action_buttons.items():
            button.clicked.connect(lambda checked, action=action: self.open_parameterizer(action))
            action_buttons_layout.addWidget(button)

        self.layout.addLayout(action_buttons_layout)

        # ---------------------------- PROGRAM AREA ----------------------------
            
        self.program_area = QGroupBox("Program Area: [Action Class] - [Actionblock Name] - [Parameters]")
        self.program_layout = QVBoxLayout()
        self.program_layout.setAlignment(QtCore.Qt.AlignTop)
        self.program_area.setLayout(self.program_layout) 

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

        # Add "Load Program" button to the top toolbar
        self.load_program_button = QPushButton("Load Program")
        self.load_program_button.clicked.connect(self.load_program)
        self.top_toolbar.addWidget(self.load_program_button)

        # Create the "Delete Program" button
        self.delete_program_button = QPushButton("Delete Program")
        self.delete_program_button.clicked.connect(self.delete_program)
        self.top_toolbar.addWidget(self.delete_program_button)

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

        # Button to spawn component
        self.spawn_component_button = QPushButton("Spawn Component", self)
        self.spawn_component_button.clicked.connect(self.spawn_component_function)
        self.top_toolbar.addWidget(self.spawn_component_button)

        # Button to destroy component
        self.spawn_component_button = QPushButton("Destroy Component", self)
        self.spawn_component_button.clicked.connect(self.destroy_component_function)
        self.top_toolbar.addWidget(self.spawn_component_button)

        #Spacer for "Spawn Component"-Button
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.top_toolbar.addWidget(spacer)

        # Button to create Grasping point
        self.spawn_component_button = QPushButton("Create Grasping Point", self)
        self.spawn_component_button.clicked.connect(self.create_grasping_point)
        self.top_toolbar.addWidget(self.spawn_component_button)

        # Button to create Grasping point
        self.spawn_component_button = QPushButton("Create Place Point", self)
        self.spawn_component_button.clicked.connect(self.create_place_point)
        self.top_toolbar.addWidget(self.spawn_component_button)

        # Button to create Home point
        self.spawn_component_button = QPushButton("Home", self)
        self.spawn_component_button.clicked.connect(self.create_home)
        self.top_toolbar.addWidget(self.spawn_component_button)

        #Spacer for "Spawn Component"-Button
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.top_toolbar.addWidget(spacer)

        # # Add the toggle button for "Simulation Mode" and "Real Mode" to the top menu
        # self.mode_toggle_group = QButtonGroup(self)
        # self.simulation_mode_button = QRadioButton("Simulation Mode")
        # self.real_mode_button = QRadioButton("Real Mode")
        # self.mode_toggle_group.addButton(self.simulation_mode_button)
        # self.mode_toggle_group.addButton(self.real_mode_button)
        # self.top_toolbar.addWidget(self.simulation_mode_button)
        # self.top_toolbar.addWidget(self.real_mode_button)

        # # Connect the mode_toggle_group to a function to handle mode changes
        # self.mode_toggle_group.buttonClicked.connect(self.handle_mode_change)

        # # Set "Simulation Mode" as the default mode
        # self.simulation_mode_button.setChecked(True)
        # self.handle_mode_change(show_info=False)

    # ---------------------------- FUNCTIONS ----------------------------

    # opens different parameterizer windows for the different ActionBlocks (instances)
    def open_parameterizer(self, action_class):
        if action_class == "MoveToFrame":
            parameterizer = MoveToFrameParameterizer(self)
            result = parameterizer.exec_()
            if result == QDialog.Accepted:
                parameters = parameterizer.save_parameters()
                action_name = parameterizer.name_edit.text()
                action_block_data = {
                    "action_class": "MoveToFrame",
                    "parameters": parameters
                }
                self.saved_action_blocks[action_name] = action_block_data
                self.update_program_area()
        elif action_class == "MoveAbsolute":
            parameterizer = MoveAbsoluteParameterizer(self)
            result = parameterizer.exec_()
            if result == QDialog.Accepted:
                parameters = parameterizer.save_parameters()
                action_name = parameterizer.name_edit.text()
                action_block_data = {
                    "action_class": "MoveAbsolute",
                    "parameters": parameters
                }
                self.saved_action_blocks[action_name] = action_block_data
                self.update_program_area()
        elif action_class == "MoveRelative":
            parameterizer = MoveRelativeParameterizer(self)
            result = parameterizer.exec_()
            if result == QDialog.Accepted:
                parameters = parameterizer.save_parameters()
                action_name = parameterizer.name_edit.text()
                action_block_data = {
                    "action_class": "MoveRelative",
                    "parameters": parameters
                }
                self.saved_action_blocks[action_name] = action_block_data
                self.update_program_area()
        elif action_class == "Grasp":
            parameterizer = GraspParameterizer(self)
            result = parameterizer.exec_()
            if result == QDialog.Accepted:
                parameters = parameterizer.save_parameters()
                action_name = parameterizer.name_edit.text()
                action_block_data = {
                    "action_class": "Grasp",
                    "parameters": parameters
                }
                self.saved_action_blocks[action_name] = action_block_data
                self.update_program_area()
        elif action_class == "Release":
            parameterizer = ReleaseParameterizer(self)
            result = parameterizer.exec_()
            if result == QDialog.Accepted:
                parameters = parameterizer.save_parameters()
                action_name = parameterizer.name_edit.text()
                action_block_data = {
                    "action_class": "Release",
                    "parameters": parameters
                }
                self.saved_action_blocks[action_name] = action_block_data
                self.update_program_area()

    # --------------------------------------------------------
    # following functions are only for the gripper actions

    def spawn_component_function(self):
        spawn_object_client = SpawnComponentClient()

        # Define the parameters for the service call
        obj_name = "Test_Component"
        parent_frame = "Gonio_Right_Part_Origin" # Drop_Position_Gonio_Left / Gonio_Right_Part_Origin
        translation = [0.0, 0.0, 0.0]
        rotation = [0.0, 0.0, 0.0, 1.0]
        cad_data = "/home/leonkoch/Distrobox/Ubuntu22/ros2_ws/src/pm_robot_programming_framework/programming_fw_pkg/programming_fw_pkg/S_UFC.STL"

        # Call the service
        spawn_object_client.spawn_component(obj_name, parent_frame, translation, rotation, cad_data)

    def destroy_component_function(self):
        destroy_object_client = DestroyComponentClient()

        # Define the parameters for the service call
        obj_name = "Test_Component"

        # Call the service 
        destroy_object_client.destroy_component(obj_name)

    def create_grasping_point(self):
        create_grasping_point_client = GripPosActionClient()
        
        # Define the parameters for the service call
        target_component = "Test_Component"

        # Call the service 
        create_grasping_point_client.send_create_ref_frame_request(target_component)
        pass

    def create_place_point(self):
        create_place_point_client = PlacePosActionClient()
        
        # Define the parameters for the service call
        target_component = "Gonio_Left_Part_Origin"

        # Call the service 
        create_place_point_client.send_create_ref_frame_request(target_component)
        pass

    def create_home(self):
        create_home_point_client = HomePosActionClient()
        
        # Define the parameters for the service call
        target_component = "Camera_Station_TCP"

        # Call the service 
        create_home_point_client.send_create_ref_frame_request(target_component)
        pass

    # --------------------------------------------------------
    # currently not used toggle button
    # def handle_mode_change(self, show_info=True):
    #     # launch sim or hw file, dont know how to handle it in the GUI...
    #     mode = "Simulation Mode" if self.simulation_mode_button.isChecked() else "Real Mode"
    #     if mode == "Real Mode":
    #         # not working yet, i dont know why!
    #         # self.destroy_component_function()
    #         print("Real Mode is selected.")
    #     else:
    #         # self.spawn_component_function
    #         print("Simulation Mode is selected.")
    #     if show_info:
    #         QMessageBox.information(self, "Mode Change", f"Switched to {mode}")

    # --------------------------------------------------------
    # define all essential functions from imported files
    
    def update_program_area(self):
        update_program_area(self)

    def save_program(self):
        save_program(self)

    def load_program(self):
        load_program(self)

    def delete_program(self):
        delete_program(self)

    def execute_program(self):
        execute_program(self)

    def clear_program(self):
        clear_program(self)

    def save_programs_to_file(self):
        save_programs_to_file(self)