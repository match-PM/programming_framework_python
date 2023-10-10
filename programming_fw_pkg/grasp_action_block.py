from action_block import ActionBlockParameterizer
from PyQt5.QtWidgets import QComboBox


class GraspParameterizer(ActionBlockParameterizer):
    def __init__(self, parent=None, action_name=None):
        super().__init__(parent, action_name)
        self.setWindowTitle("Parameterize 'Gripper Grasp'-Actionblock")

        self.create_parameterizer()

    def create_parameterizer(self):     

        self.grasp_tool_combo = QComboBox()
        self.grasp_tool_combo.addItems(["Vacuum_Tool", "Standard_Tool"])

        self.target_component_combo = QComboBox()
        self.target_component_combo.addItems(["Test_Component"])

        self.form_layout.addRow("Tool:", self.grasp_tool_combo)
        self.form_layout.addRow("Target Component:", self.target_component_combo)

    def save_parameters(self):
        parameters = {
            "Tool": self.grasp_tool_combo.currentText(),
            "TargetComponent": self.target_component_combo.currentText()
        }

        # Store the parameters in a dictionary for the action block
        action_block_data = {
            "action_class": "Grasp",
            "parameters": parameters
        }

        # Add the action block data to the parent's saved_action_blocks dictionary
        self.parent().saved_action_blocks[self.name_edit.text()] = action_block_data
        
        self.accept()

        return parameters

    
    