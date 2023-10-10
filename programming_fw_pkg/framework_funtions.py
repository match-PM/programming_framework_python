import rclpy
from PyQt5.QtWidgets import QLabel, QFrame
from move_to_frame_service_client import MoveToFrameServiceClient
from move_absolute_service_client import MoveAbsoluteServiceClient
from move_relative_service_client import MoveRelativeServiceClient
from grasp_service_client import GraspServiceClient
from release_service_client import ReleaseServiceClient
import time

ACTION_CLASS_TO_CLIENT = {
    "MoveToFrame": MoveToFrameServiceClient,
    "MoveAbsolute": MoveAbsoluteServiceClient,
    "MoveRelative": MoveRelativeServiceClient,
    "Grasp": GraspServiceClient,
    "Release": ReleaseServiceClient,
    # add other action classes and clients here
}

def update_program_area(self):
    # Clear the existing layout to always have the current saved action blocks
    for i in reversed(range(self.saved_action_blocks_layout.count())):
        self.saved_action_blocks_layout.itemAt(i).widget().setParent(None)

    # Add the saved ActionBlocks to the program layout (= Program Area)
    for action_name, action_block_data in self.saved_action_blocks.items():
        action_class = action_block_data["action_class"]
        parameters = action_block_data["parameters"]
        if parameters:
            parameter_str = ", ".join(f"{key}: {value}" for key, value in parameters.items())
            action_label = QLabel(f"{action_class} - {action_name} - {parameter_str}")
        else:
            action_label = QLabel(f"{action_class} - {action_name}")

        # Add a visual border around the action block
        action_label.setFrameShape(QFrame.Panel)
        action_label.setFrameShadow(QFrame.Sunken)

        self.saved_action_blocks_layout.addWidget(action_label)

        # for debugging purposes:
        # print("Contents of saved_action_blocks:", self.saved_action_blocks)

def execute_program(self):
    rclpy.init()

    action_names = list(self.saved_action_blocks.keys())

    client_instances = {}  # Store the instances of all service clients

    def callback(future):
        nonlocal action_name
        self.handle_response(future)  # handle the response
        action_name = next(action_names_iter, None)
        if action_name:
            client_instances[action_name].send_request()

    action_names_iter = iter(action_names)
    action_name = next(action_names_iter, None)

    while action_name:
        action_block = self.saved_action_blocks[action_name]
        action_class = action_block["action_class"]

        if action_class in ACTION_CLASS_TO_CLIENT:
            if action_name not in client_instances:
                client_class = ACTION_CLASS_TO_CLIENT[action_class]
                client = client_class(action_block)
                client_instances[action_name] = client

            client.set_callback(callback)
            print(f"Calling client for action: {action_name}, class: {action_class}") # for debugging purposes
            client.send_request()  # Initiating the service request

            time.sleep(1)

        action_name = next(action_names_iter, None)  # Move to the next action block

    rclpy.shutdown()

def clear_program(self):
    for i in reversed(range(self.saved_action_blocks_layout.count())):
        self.saved_action_blocks_layout.itemAt(i).widget().setParent(None)

    # Clear the saved action blocks dic
    self.saved_action_blocks = {}