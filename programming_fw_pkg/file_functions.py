import json
from framework_funtions import clear_program, update_program_area
from PyQt5.QtWidgets import QMessageBox, QInputDialog



def save_program(self):
    program_name, _ = QInputDialog.getText(self, "Save Program", "Enter a name for the program:")
    if program_name:
        saved_data = {"programs": self.programs}
        saved_data["programs"][program_name] = self.saved_action_blocks
        with open("programs.json", "w") as json_file:
            json.dump(saved_data, json_file, indent=4)
    for i in reversed(range(self.saved_action_blocks_layout.count())):
        self.saved_action_blocks_layout.itemAt(i).widget().setParent(None)
        
    # Clear the saved action blocks and parameters
    self.saved_action_blocks = {}

def load_program(self):
    if not self.programs:
        QMessageBox.information(self, "No Programs found", "No programs to load.")
        return
    with open("programs.json", "r") as json_file:
        saved_data = json.load(json_file)
        self.programs = saved_data.get("programs", {})  # Load into self.programs
        program_names = list(self.programs.keys())
        program_name, ok = QInputDialog.getItem(self, "Load Program", "Select a program to load:", program_names, editable=False)
        if ok and program_name:
            self.saved_action_blocks = self.programs.get(program_name, [])
            update_program_area(self)

def delete_program(self):
    if not self.programs:
        QMessageBox.information(self, "No Programs found", "No programs to delete.")
        return
    program_names = list(self.programs.keys())
    program_name, ok = QInputDialog.getItem(self, "Delete Program", "Select a program to delete:", program_names, editable=False)
    if ok and program_name:
        del self.programs[program_name]
        update_program_area(self)
        save_programs_to_file(self)  # Save updated programs to file
    clear_program(self)

def save_programs_to_file(self):
    saved_data = {"programs": self.programs}
    with open("programs.json", "w") as json_file:
        json.dump(saved_data, json_file, indent=4)

def load_programs_from_file(self):
    try:
        with open("programs.json", "r") as json_file:
            saved_data = json.load(json_file)
            self.programs = saved_data.get("programs", {})
    except FileNotFoundError:
        self.programs = {}