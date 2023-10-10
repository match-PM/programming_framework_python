from action_block import ActionBlockParameterizer

from PyQt5.QtWidgets import QLineEdit, QComboBox, QCheckBox


class MoveAbsoluteParameterizer(ActionBlockParameterizer):
    def __init__(self, parent=None, action_name=None):
        super().__init__(parent, action_name)
        self.setWindowTitle("Parameterize 'Move Absolute'-Actionblock")

        self.create_parameterizer()

    def create_parameterizer(self):     
        self.move_group_combo = QComboBox()
        self.move_group_combo.addItems(["Tool", "Cam1", "Laser"])

        self.position_edit = QLineEdit()
        self.position_edit.setText("0.0, 0.0, 0.0")
        self.orientation_edit = QLineEdit()
        self.orientation_edit.setText("0.0, 0.0, 0.0, 1.0")

        self.wait_input_checkbox = QCheckBox("Wait for User Input")

        self.form_layout.addRow("MoveGroup:", self.move_group_combo)
        self.form_layout.addRow("Position (x, y, z):", self.position_edit)
        self.form_layout.addRow("Orientation (x, y, z, w):", self.orientation_edit)
        self.form_layout.addRow(self.wait_input_checkbox)

    def save_parameters(self):
        parameters = {
            "MoveGroup": self.move_group_combo.currentText(),
            "Position (x, y, z)": self.position_edit.text(),
            "Orientation (x, y, z, w)": self.orientation_edit.text(),
            "Wait for User Input": self.wait_input_checkbox.isChecked()
        }

        # Store the parameters in a dictionary for the action block
        action_block_data = {
            "action_class": "MoveToFrame",
            "parameters": parameters
        }

        # Add the action block data to the parent's saved_action_blocks dictionary
        self.parent().saved_action_blocks[self.name_edit.text()] = action_block_data
        
        self.accept()

        return parameters
    
    