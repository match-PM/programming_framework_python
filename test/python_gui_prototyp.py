import tkinter as tk
from tkinter import messagebox, ttk
import subprocess

class ROS2ServiceGUI(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title('PM_programming_framework')
        self.geometry('300x650')

        #Create a list of the MoveGroups
        move_group = ['Cam1', 'Laser', 'Tool']
        # Create a list of frames for the dropdown menu
        target_frames = ["Camera_Station_TCP", "Gonio_Left_Part_Origin", "Gonio_Right_Part_Origin", "Test_Station", "Station_Laser_Bottom_TCP"]
        #target_frames = ['Camera_Station_TCP', 'Tool_TCP', 'Laser']

        self.heading_label = tk.Label(self, text='Simulation or Real-Mode')
        self.heading_label.pack(pady=10)
        # Create a custom toggle button to switch between Simulation and Real modes
        self.mode_var = tk.StringVar()
        self.mode_var.set('Simulation')
        self.mode_button = ttk.Button(self, textvariable=self.mode_var, width=15, command=self.toggle_mode)
        self.mode_button.pack(pady=5)

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

        # Create the "Call Service" button
        self.call_service_button = tk.Button(self, text='Execute', command=self.call_service)
        self.call_service_button.pack(pady=10)

    def toggle_mode(self):
       current_mode = self.mode_var.get()
       if current_mode == 'Simulation':
           self.mode_var.set('Real')
       else:
           self.mode_var.set('Simulation')

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
        
        # Pepare the command to call the service node

        #command = f'python3 {node_name} "{frame_name}" {position_x} {position_y} {position_z} {orientation_x} {orientation_y} {orientation_z} {orientation_w} {translation_x} {translation_y} {translation_z} {exec_wait_for_user_input}'
        #command = f'python3 call_service.py "{frame_name}" {position_x} {position_y} {position_z} {orientation_x} {orientation_y} {orientation_z} {orientation_w} {translation_x} {translation_y} {translation_z} {exec_wait_for_user_input}'
        
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
        #exept ValueError:
        #   messagebox.showerror('Error', 'Invalid input. Please enter valid numeric values.')

if __name__ == '__main__':
    app = ROS2ServiceGUI()
    app.mainloop()

# ros2 service call /pm_moveit_server/move_tool_to_frame pm_moveit_interfaces/srv/MoveToolTcpTo "{
#   frame_name: 'Camera_Station_TCP',
#   move_to_pose: {
#     position: {
#       x: 0.530,
#       y: 0.40,
#       z: 1.3
#     },
#     orientation: {
#       x: 0.0,
#       y: 0.0,
#       z: 0.0,
#       w: 1.0
#     }},
#     translation: {
#       x: 0.0,
#       y: 0.0,
#       z: 0.015
#     },
#     exec_wait_for_user_input: false,
#     execute: true
 
# }"

# ros2 service call /pm_moveit_server/move_tool_to_frame pm_moveit_interfaces/srv/MoveToolTcpTo "{frame_name: 'Camera_Station_TCP', translation: {x: 0.0, y: 0.0, z: 0.015}, execute: true }"

#only fo to camera frame
#ros2 service call /pm_moveit_server/move_tool_to_frame pm_moveit_interfaces/srv/MoveToolTcpTo "{frame_name: 'Camera_Station_TCP', execute: true}"

#ros2 service call /pm_moveit_server/move_tool_to_frame pm_moveit_interfaces/srv/MoveToolTcpTo "{frame_name: 'Gonio_Right_Part_Origin', execute: true}"
