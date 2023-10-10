# import tkinter as tk
# from tkinter import messagebox, ttk
# import subprocess

# class ActionBlockParameterWindow(tk.Toplevel):
#     def __init__(self, action_block, main_app):
#         super().__init__()
#         self.title(f'Parameterize {action_block.name}')
#         self.geometry('400x650')

#         self.action_block = action_block  # Store the action block
#         self.main_app = main_app  # Store a reference to the main application

#         # Add a name_label and name_entry for the action block name
#         self.name_label = tk.Label(self, text='Action Block Name:')
#         self.name_label.pack(pady=5)
#         self.name_entry = tk.Entry(self)
#         self.name_entry.pack()

#         #Create a list of the MoveGroups
#         move_group = ['Cam1', 'Laser', 'Tool']
#         # Create a list of frames for the dropdown menu
#         target_frames = ['Camera_Station_TCP', 'Tool_TCP', 'Laser']

#         # Create a label and dropdown menu for MoveGroup selection
#         self.move_group_label = tk.Label(self, text='Select MoveGroup:')
#         self.move_group_label.pack(pady=5)
#         self.move_group_var = tk.StringVar()
#         self.move_group_var.set(move_group[0])  # Set the default MoveGroup
#         self.move_group_menu = ttk.Combobox(self, textvariable=self.move_group_var, values=move_group)
#         self.move_group_menu.pack(pady=5)

#         # Create a label and dropdown menu for frame selection
#         self.frame_name_label = tk.Label(self, text='Select Targetframe:')
#         self.frame_name_label.pack(pady=5)
#         self.frame_name_var = tk.StringVar()
#         self.frame_name_var.set(target_frames[0])  # Set the default frame
#         self.frame_name_menu = ttk.Combobox(self, textvariable=self.frame_name_var, values=target_frames)
#         self.frame_name_menu.pack(pady=5)

#         self.position_label = tk.Label(self, text='Position (x, y, z):')
#         self.position_label.pack(pady=5)
#         self.position_x_entry = tk.Entry(self)
#         self.position_x_entry.pack()
#         self.position_y_entry = tk.Entry(self)
#         self.position_y_entry.pack()
#         self.position_z_entry = tk.Entry(self)
#         self.position_z_entry.pack()

#         self.orientation_label = tk.Label(self, text='Orientation (x, y, z, w):')
#         self.orientation_label.pack(pady=5)
#         self.orientation_x_entry = tk.Entry(self)
#         self.orientation_x_entry.pack()
#         self.orientation_y_entry = tk.Entry(self)
#         self.orientation_y_entry.pack()
#         self.orientation_z_entry = tk.Entry(self)
#         self.orientation_z_entry.pack()
#         self.orientation_w_entry = tk.Entry(self)
#         self.orientation_w_entry.pack()

#         self.translation_label = tk.Label(self, text='Translation (x, y, z):')
#         self.translation_label.pack(pady=5)
#         self.translation_x_entry = tk.Entry(self)
#         self.translation_x_entry.pack()
#         self.translation_y_entry = tk.Entry(self)
#         self.translation_y_entry.pack()
#         self.translation_z_entry = tk.Entry(self)
#         self.translation_z_entry.pack()

#         self.exec_wait_var = tk.BooleanVar()
#         self.exec_wait_var.set(False)
#         self.exec_wait_checkbox = tk.Checkbutton(self, text='Wait for User Input', variable=self.exec_wait_var)
#         self.exec_wait_checkbox.pack(pady=5)

#         self.back_button = tk.Button(self, text='Back', command=self.destroy)
#         self.back_button.pack(pady=5)

#         self.save_button = tk.Button(self, text='Save', command=self.save_parameters)
#         self.save_button.pack(pady=5)

#     def save_parameters(self):
#         action_block_name = self.name_entry.get()
#         if not action_block_name:
#             messagebox.showerror('Error', 'Please enter a name for the action block.')
#             return
        
#         params = {
#             'MoveGroup': self.move_group_var.get(),
#             'TargetFrame': self.frame_name_var.get(),
#             'Position (X)': self.position_x_entry.get(),
#             'Position (Y)': self.position_y_entry.get(),
#             'Position (Z)': self.position_z_entry.get(),
#             'Orientation (X)': self.orientation_x_entry.get(),
#             'Orientation (Y)': self.orientation_y_entry.get(),
#             'Orientation (Z)': self.orientation_z_entry.get(),
#             'Orientation (W)': self.orientation_w_entry.get(),
#             'Translation (X)': self.translation_x_entry.get(),
#             'Translation (Y)': self.translation_y_entry.get(),
#             'Translation (Z)': self.translation_z_entry.get(),
#             'Wait for User Input': self.exec_wait_var.get()
#         }

#         print(f"Action Block Name: {action_block_name}")
#         print(f"Action Block Type: {self.action_block.name}")
#         for param_name, param_value in params.items():
#             print(f"{param_name}: {param_value}")

#         # Save the parameters in the action block
#         self.action_block.name = action_block_name
#         self.action_block.parameter_values = params

#         # Update the main application display
#         self.main_app.display_action_block_parameters()
#         self.destroy()

# class ActionBlock:
#     def __init__(self, name, parameters):
#         self.name = name
#         self.parameters = parameters
#         self.parameter_values = {}  # Store the parameter values in this dictionary

# class ProgrammingFramework(tk.Tk):
#     def __init__(self):
#         super().__init__()
#         self.title('PM_programming_framework')
#         self.geometry('800x800')

#         self.heading_label = tk.Label(self, text='Simulation or Real-Mode')
#         self.heading_label.pack(pady=10)
#         # Create a custom toggle button to switch between Simulation and Real modes
#         self.mode_var = tk.StringVar()
#         self.mode_var.set('Simulation')
#         self.mode_button = ttk.Button(self, textvariable=self.mode_var, width=15, command=self.toggle_mode)
#         self.mode_button.pack(pady=5)

#         # Left side (Action Blocks)
#         self.left_frame = tk.Frame(self, width=400, height=800, bg='lightgray')
#         self.left_frame.pack(side='left', fill='both', expand=True)

#         # Create action blocks
#         self.action_blocks = []
#         self.create_action_blocks()

#         # Right side (Display Action Block Parameters)
#         self.right_frame = tk.Frame(self, width=400, height=800, bg='white')
#         self.right_frame.pack(side='right', fill='both', expand=True)
#         self.action_block_parameters_label = tk.Label(self.right_frame, text='Action Block Parameters', font=('Arial', 14, 'bold'))
#         self.action_block_parameters_label.pack(pady=10)
#         self.action_block_parameters_display = tk.Text(self.right_frame, wrap=tk.WORD, bg='white')
#         self.action_block_parameters_display.pack(pady=5, padx=10, fill='both', expand=True)

#         # Create the "Call Service" button
#         self.call_service_button = tk.Button(self, text='Execute', command=self.call_service)
#         self.call_service_button.pack(pady=10)

#     def toggle_mode(self):
#         current_mode = self.mode_var.get()
#         if current_mode == 'Simulation':
#             self.mode_var.set('Real')
#         else:
#             self.mode_var.set('Simulation')

#     def create_action_blocks(self):
#         move_to_frame_params = {
#             'MoveGroup': 'Select MoveGroup',
#             'TargetFrame': 'Select Targetframe',
#             'Position (X)': 'Position (X)',
#             'Position (Y)': 'Position (Y)',
#             'Position (Z)': 'Position (Z)',
#             'Orientation (X)': 'Orientation (X)',
#             'Orientation (Y)': 'Orientation (Y)',
#             'Orientation (Z)': 'Orientation (Z)',
#             'Orientation (W)': 'Orientation (W)',
#             'Translation (X)': 'Translation (X)',
#             'Translation (Y)': 'Translation (Y)',
#             'Translation (Z)': 'Translation (Z)',
#             'Wait for User Input': 'Wait for User Input'
#         }
#         move_to_frame_block = ActionBlock('MoveToFrame', move_to_frame_params)
#         self.action_blocks.append(move_to_frame_block)

#         # Create buttons for action blocks
#         for action_block in self.action_blocks:
#             action_block_button = ttk.Button(self.left_frame, text=action_block.name, command=lambda block=action_block: self.open_parameter_window(block))
#             action_block_button.pack(pady=5)

#     def open_parameter_window(self, action_block):
#         parameter_window = ActionBlockParameterWindow(action_block, self)
#         parameter_window.mainloop()

#     def display_action_block_parameters(self):
#         # Clear the existing display
#         self.action_block_parameters_display.delete('1.0', tk.END)

#         # Iterate over all action blocks and update the display with their parameters
#         for action_block in self.action_blocks:
#             self.action_block_parameters_display.insert(tk.END, f"Action Block Name: {action_block.name}\n")
#             for param_name, param_value in action_block.parameter_values.items():
#                 self.action_block_parameters_display.insert(tk.END, f"{param_name}: {param_value}\n")
#             self.action_block_parameters_display.insert(tk.END, "\n")

#     def call_service(self):
#         try:
#             move_group = self.move_group_var.get()

#             frame_name = "'" + self.frame_name_var.get() + "'" # Enclose frame name in single quotes
#             position_x = self.position_x_entry.get()
#             position_y = self.position_y_entry.get()
#             position_z = self.position_z_entry.get()
#             orientation_x = self.orientation_x_entry.get()
#             orientation_y = self.orientation_y_entry.get()
#             orientation_z = self.orientation_z_entry.get()
#             orientation_w = self.orientation_w_entry.get()
#             translation_x = self.translation_x_entry.get()
#             translation_y = self.translation_y_entry.get()
#             translation_z = self.translation_z_entry.get()
#             exec_wait_for_user_input = self.exec_wait_var.get()
#         except ValueError:
#             messagebox.showerror('Error', 'Invalid input. Please enter valid numeric values.')
#             return
        
#          # Determine the node to call based on the selected MoveGroup
#         if move_group == 'Cam1':
#             node_name = 'movegroup_cam1_call_service.py'
#         elif move_group == 'Laser':
#             node_name = 'movegroup_laser_call_service.py'
#         elif move_group == 'Tool':
#             node_name = 'movegroup_tool_call_service.py'
#         else:
#             messagebox.showerror('Error', 'Invalid MoveGroup selection.')
#             return
        
#         # Prepare the command to call the service node
#         command = f'python3 {node_name} --frame_name={frame_name}'

#         # Add parameters to the command string only if they are not empty
#         if position_x:
#             command += f' --position_x={position_x}'
#         if position_y:
#             command += f' --position_y={position_y}'
#         if position_z:
#             command += f' --position_z={position_z}'
#         if orientation_x:
#             command += f' --orientation_x={orientation_x}'
#         if orientation_y:
#             command += f' --orientation_y={orientation_y}'
#         if orientation_z:
#             command += f' --orientation_z={orientation_z}'
#         if orientation_w:
#             command += f' --orientation_w={orientation_w}'
#         if translation_x:
#             command += f' --translation_x={translation_x}'
#         if translation_y:
#             command += f'--translation_y={translation_y}'
#         if translation_z:
#             command += f'--translation_z={translation_z}'
#         if exec_wait_for_user_input:
#             command += '--exec_wait_for_user_input'

#         try:
#         # Call the service node using subprocess
#             subprocess.run(command, shell=True, check=True)
#             messagebox.showinfo('Success', 'Service call successful.')
#         except subprocess.CalledProcessError:
#             messagebox.showerror('Error', 'Service call failed.')


# if __name__ == '__main__':
#     app = ProgrammingFramework()
#     app.mainloop()

import tkinter as tk
from tkinter import messagebox, ttk
import subprocess

class ActionBlockParameterWindow(tk.Toplevel):
    def __init__(self, action_block, main_app):
        super().__init__()
        self.title(f'Parameterize {action_block.name}')
        self.geometry('400x650')

        self.action_block = action_block  # Store the action block
        self.main_app = main_app  # Reference to the main app

        # Add a name_label and name_entry for the action block name
        self.name_label = tk.Label(self, text='Action Block Name:')
        self.name_label.pack(pady=5)
        self.name_entry = tk.Entry(self)
        self.name_entry.pack()

        #Create a list of the MoveGroups
        move_group = ['Cam1', 'Laser', 'Tool']
        # Create a list of frames for the dropdown menu
        target_frames = ['Camera_Station_TCP', 'Tool_TCP', 'Laser']

        # Create a label and dropdown menu for MoveGroup selection
        self.move_group_label = tk.Label(self, text='Select MoveGroup:')
        self.move_group_label.pack(pady=5)
        self.move_group_var = tk.StringVar()
        self.move_group_var.set(move_group[0])  # Set the default MoveGroup
        self.move_group_menu = ttk.Combobox(self, textvariable=self.move_group_var, values=move_group)
        self.move_group_menu.pack(pady=5)

        # Create a label and dropdown menu for frame selection
        self.frame_name_label = tk.Label(self, text='Select Targetframe:')
        self.frame_name_label.pack(pady=5)
        self.frame_name_var = tk.StringVar()
        self.frame_name_var.set(target_frames[0])  # Set the default frame
        self.frame_name_menu = ttk.Combobox(self, textvariable=self.frame_name_var, values=target_frames)
        self.frame_name_menu.pack(pady=5)

        self.position_label = tk.Label(self, text='Position (x, y, z):')
        self.position_label.pack(pady=5)
        self.position_x_entry = tk.Entry(self)
        self.position_x_entry.pack()
        self.position_y_entry = tk.Entry(self)
        self.position_y_entry.pack()
        self.position_z_entry = tk.Entry(self)
        self.position_z_entry.pack()

        self.orientation_label = tk.Label(self, text='Orientation (x, y, z, w):')
        self.orientation_label.pack(pady=5)
        self.orientation_x_entry = tk.Entry(self)
        self.orientation_x_entry.pack()
        self.orientation_y_entry = tk.Entry(self)
        self.orientation_y_entry.pack()
        self.orientation_z_entry = tk.Entry(self)
        self.orientation_z_entry.pack()
        self.orientation_w_entry = tk.Entry(self)
        self.orientation_w_entry.pack()

        self.translation_label = tk.Label(self, text='Translation (x, y, z):')
        self.translation_label.pack(pady=5)
        self.translation_x_entry = tk.Entry(self)
        self.translation_x_entry.pack()
        self.translation_y_entry = tk.Entry(self)
        self.translation_y_entry.pack()
        self.translation_z_entry = tk.Entry(self)
        self.translation_z_entry.pack()

        self.exec_wait_var = tk.BooleanVar()
        self.exec_wait_var.set(False)
        self.exec_wait_checkbox = tk.Checkbutton(self, text='Wait for User Input', variable=self.exec_wait_var)
        self.exec_wait_checkbox.pack(pady=5)

        self.back_button = tk.Button(self, text='Back', command=self.destroy)
        self.back_button.pack(pady=5)

        self.save_button = tk.Button(self, text='Save', command=self.save_parameters)
        self.save_button.pack(pady=5)


    def save_parameters(self):
        action_block_name = self.name_entry.get()
        if not action_block_name:
            messagebox.showerror('Error', 'Please enter a name for the action block.')
            return

        params = {
            'Move Group': self.move_group_var.get(),
            'Target Frame': self.frame_name_var.get(),
            'Position (x, y, z)': f"{self.position_x_entry.get()}, {self.position_y_entry.get()}, {self.position_z_entry.get()}",
            'Orientation (x, y, z, w)': f"{self.orientation_x_entry.get()}, {self.orientation_y_entry.get()}, {self.orientation_z_entry.get()}, {self.orientation_w_entry.get()}",
            'Translation (x, y, z)': f"{self.translation_x_entry.get()}, {self.translation_y_entry.get()}, {self.translation_z_entry.get()}",
            'Wait for User Input': 'Yes' if self.exec_wait_var.get() else 'No',
        }

        # Save the action block with the entered name and parameters here
        new_action_block = ActionBlock(action_block_name, params)
        self.main_app.add_action_block(new_action_block)

        self.destroy()

class ActionBlock:
    def __init__(self, name, parameters):
        self.name = name
        self.parameters = parameters

class ProgrammingFramework(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('PM_programming_framework')
        self.geometry('1000x1000')

        self.heading_label = tk.Label(self, text='Simulation or Real-Mode')
        self.heading_label.pack(pady=10)
        # Create a custom toggle button to switch between Simulation and Real modes
        self.mode_var = tk.StringVar()
        self.mode_var.set('Simulation')
        self.mode_button = ttk.Button(self, textvariable=self.mode_var, width=15, command=self.toggle_mode)
        self.mode_button.pack(pady=5)

        # Left side (Action Blocks)
        self.left_frame = tk.Frame(self, width=400, height=800, bg='lightgray')
        self.left_frame.pack(side='left', fill='both', expand=True)

        # Create action blocks
        self.action_blocks = []
        self.create_action_blocks()

        # Right side (Display Action Block Parameters)
        self.right_frame = tk.Frame(self, width=400, height=800, bg='white')
        self.right_frame.pack(side='right', fill='both', expand=True)
        self.action_block_parameters_label = tk.Label(self.right_frame, text='Program Area', font=('Arial', 14, 'bold'))
        self.action_block_parameters_label.pack(pady=10)
        self.action_block_program_display = tk.Text(self.right_frame, wrap=tk.WORD, font=('Arial', 12), bg='white')
        self.action_block_program_display.pack(pady=5, padx=10, fill='both', expand=True)

        # Create the "Execute Program" button
        self.call_service_button = tk.Button(self, text='Execute Program', command=self.call_service)
        self.call_service_button.pack(pady=10)

        self.clear_all_button = tk.Button(self, text='Clear Program', command=self.clear_all_action_blocks)
        self.clear_all_button.pack(side='right', padx=10, pady=10)

        # # # Create the text widget to display action block parameters
        # self.action_block_parameters_display = tk.Text(self.right_frame, wrap='word', font=('Arial', 12), bg='white')
        # self.action_block_parameters_display.pack(pady=5, padx=10, fill='both', expand=True)
        

    def toggle_mode(self):
        current_mode = self.mode_var.get()
        if current_mode == 'Simulation':
            self.mode_var.set('Real')
        else:
            self.mode_var.set('Simulation')

    def create_action_blocks(self):
        move_to_frame_params = {
            'Move Group': 'Select MoveGroup',
            'Target Frame': 'Select Targetframe',
            'Position (x, y, z)': '',
            'Orientation (x, y, z, w)': '',
            'Translation (x, y, z)': '',
            'Wait for User Input': False
        }
        move_to_frame_block = ActionBlock('MoveToFrame', move_to_frame_params)
        self.action_blocks.append(move_to_frame_block)

        # Create buttons for action blocks
        for action_block in self.action_blocks:
            action_block_button = ttk.Button(self.left_frame, text=action_block.name, command=lambda block=action_block: self.open_parameter_window(block))
            action_block_button.pack(pady=5)

    def open_parameter_window(self, action_block):
        parameter_window = ActionBlockParameterWindow(action_block, self)
        parameter_window.mainloop()

    def add_action_block(self, action_block):
        self.action_blocks.append(action_block)
        self.display_action_block_parameters()

    def display_action_block_parameters(self):
        self.action_block_program_display.delete('1.0', tk.END)
        for action_block in self.action_blocks:
            self.action_block_program_display.insert(tk.END, f"{action_block.name}:\n")
            for param_name, param_value in action_block.parameters.items():
                self.action_block_program_display.insert(tk.END, f"{param_name}: {param_value}\n")
            self.action_block_program_display.insert(tk.END, "\n")

    def clear_all_action_blocks(self):
        self.action_blocks.clear()
        self.display_action_block_parameters()

    def call_service(self):
        try:
            move_group = self.move_group_var.get()

            frame_name = "'" + self.frame_name_var.get() + "'" # Enclose frame name in single quotes
            position_x = self.position_x_entry.get()
            position_y = self.position_y_entry.get()
            position_z = self.position_z_entry.get()
            orientation_x = self.orientation_x_entry.get()
            orientation_y = self.orientation_y_entry.get()
            orientation_z = self.orientation_z_entry.get()
            orientation_w = self.orientation_w_entry.get()
            translation_x = self.translation_x_entry.get()
            translation_y = self.translation_y_entry.get()
            translation_z = self.translation_z_entry.get()
            exec_wait_for_user_input = self.exec_wait_var.get()
        except ValueError:
            messagebox.showerror('Error', 'Invalid input. Please enter valid numeric values.')
            return
        
         # Determine the node to call based on the selected MoveGroup
        if move_group == 'Cam1':
            node_name = 'movegroup_cam1_call_service.py'
        elif move_group == 'Laser':
            node_name = 'movegroup_laser_call_service.py'
        elif move_group == 'Tool':
            node_name = 'movegroup_tool_call_service.py'
        else:
            messagebox.showerror('Error', 'Invalid MoveGroup selection.')
            return
        
        # Prepare the command to call the service node
        command = f'python3 {node_name} --frame_name={frame_name}'

        # Add parameters to the command string only if they are not empty
        if position_x:
            command += f' --position_x={position_x}'
        if position_y:
            command += f' --position_y={position_y}'
        if position_z:
            command += f' --position_z={position_z}'
        if orientation_x:
            command += f' --orientation_x={orientation_x}'
        if orientation_y:
            command += f' --orientation_y={orientation_y}'
        if orientation_z:
            command += f' --orientation_z={orientation_z}'
        if orientation_w:
            command += f' --orientation_w={orientation_w}'
        if translation_x:
            command += f' --translation_x={translation_x}'
        if translation_y:
            command += f'--translation_y={translation_y}'
        if translation_z:
            command += f'--translation_z={translation_z}'
        if exec_wait_for_user_input:
            command += '--exec_wait_for_user_input'

        try:
        # Call the service node using subprocess
            subprocess.run(command, shell=True, check=True)
            messagebox.showinfo('Success', 'Service call successful.')
        except subprocess.CalledProcessError:
            messagebox.showerror('Error', 'Service call failed.')

if __name__ == '__main__':
    app = ProgrammingFramework()
    app.mainloop()
