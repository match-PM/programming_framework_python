from PyQt5.QtWidgets import QDialog, QVBoxLayout, QPushButton, QHBoxLayout, QFormLayout, QLineEdit

# Parent for all Actionblocks
class ActionBlockParameterizer(QDialog):
    def __init__(self, parent, action_name=None):
        super().__init__(parent)
        self.action_name = action_name
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("ActionBlock Parameterizer")

        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("Enter an unique name for the ActionBlock")

        # Create save- and back-buttons
        self.save_button = QPushButton("Save")
        self.cancel_button = QPushButton("Cancel")
        self.save_button.clicked.connect(self.save_parameters)
        self.cancel_button.clicked.connect(self.close)
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.save_button)
        button_layout.addWidget(self.cancel_button)

        self.form_layout = QFormLayout()
        layout = QVBoxLayout()
        layout.addWidget(self.name_edit)
        layout.addLayout(self.form_layout)
        layout.addLayout(button_layout)
        self.setLayout(layout)
    