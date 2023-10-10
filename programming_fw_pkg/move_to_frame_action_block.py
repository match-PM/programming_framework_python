from action_block import ActionBlockParameterizer

from PyQt5.QtWidgets import QLineEdit, QComboBox, QCheckBox

#Move absolute need MoveGroup + Pose (no target frame!)
class MoveToFrameParameterizer(ActionBlockParameterizer):
    def __init__(self, parent=None, action_name=None):
        super().__init__(parent, action_name)
        self.setWindowTitle("Parameterize 'Move To Frame'-Actionblock")

        self.create_parameterizer()

    def create_parameterizer(self):     
        self.move_group_combo = QComboBox()
        self.move_group_combo.addItems(["Tool", "Cam1", "Laser"])
        self.target_frame_combo = QComboBox()
        self.target_frame_combo.addItems(["Home", "Drop_Position_Gonio_Left", "Camera_Station_TCP", "Gonio_Left_Part_Origin", "Gonio_Right_Part_Origin", "Test_Station", "Station_Laser_Bottom_TCP", "Grasping_Position"])
        # there are no frames for: "Tool_TCP", "Laser"
        # a bad one for: "Gonio_Left_Part_Origin"
        self.offset_edit = QLineEdit()
        self.offset_edit.setText("0.0, 0.0, 0.0")
        self.rotation_edit = QLineEdit()
        self.rotation_edit.setText("0.0, 0.0, 0.0, 1.0")

        self.wait_input_checkbox = QCheckBox("Wait for User Input")

        self.form_layout.addRow("MoveGroup:", self.move_group_combo)
        self.form_layout.addRow("TargetFrame:", self.target_frame_combo)
        self.form_layout.addRow("Offset (x, y, z):", self.offset_edit)
        self.form_layout.addRow("Rotation (x, y, z, w):", self.rotation_edit)
        self.form_layout.addRow(self.wait_input_checkbox)

    def save_parameters(self):
        parameters = {
            "MoveGroup": self.move_group_combo.currentText(),
            "TargetFrame": self.target_frame_combo.currentText(),
            "Offset (x, y, z)": self.offset_edit.text(),
            "Rotation (x, y, z, w)": self.rotation_edit.text(),
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
    
    
    