from action_block import ActionBlockParameterizer
from PyQt5.QtWidgets import QComboBox

class ReleaseParameterizer(ActionBlockParameterizer):
    def __init__(self, parent=None, action_name=None):
        super().__init__(parent, action_name)
        self.setWindowTitle("Parameterize 'Gripper Release'-Actionblock")

        self.create_parameterizer()

    def create_parameterizer(self):     
        self.target_combo = QComboBox()
        self.target_combo.addItems(["Gonio_Left_Part_Origin", "Drop_Position_Gonio_Left","Camera_Station_TCP", "Gonio_Right_Part_Origin", "Test_Station", "Station_Laser_Bottom_TCP"])

        self.target_component_combo = QComboBox()
        self.target_component_combo.addItems(["Test_Component"])

        self.form_layout.addRow("Component:", self.target_component_combo)
        self.form_layout.addRow("Target:", self.target_combo)
    
    def save_parameters(self):
        parameters = {
            "Component": self.target_component_combo.currentText(),
            "Target": self.target_combo.currentText(),
        }

        # Store the parameters in a dictionary for the action block
        action_block_data = {
            "action_class": "Release",
            "parameters": parameters
        }

        self.parent().saved_action_blocks[self.name_edit.text()] = action_block_data

        self.accept()

        return parameters

    
    
    