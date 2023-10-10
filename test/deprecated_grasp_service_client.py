import rclpy
from rclpy.node import Node
from spawn_object_interfaces.srv import CreateRefFrame, ChangeParentFrame
from pm_moveit_interfaces.srv import MoveToolTcpTo
import threading
import geometry_msgs.msg

"""
1. Spawn Ref-Frame
2. Move to Ref-Frame
3. Change Parent-Frame

ros2 service call /object_manager/create_ref_frame spawn_object_interfaces/srv/CreateRefFrame "{frame_name: Ref_Frame, parent_frame: Test_Component , pose:{position: {x: -0.01755, y: -0.010935, z: 0.002359}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

ros2 service call /pm_moveit_server/move_tool_to_frame  pm_moveit_interfaces/srv/MoveToolTcpTo "{frame_name: Ref_Frame, translation: {x: 0.0, y: 0.0, z: 0.001}, exec_wait_for_user_input: false, execute: true }"

ros2 service call /object_manager/change_obj_parent_frame spawn_object_interfaces/srv/ChangeParentFrame "{obj_name: Test_Component, parent_frame: PM_Robot_Tool_TCP }"
"""

#unique_node_counter = 0

class GraspServiceClient(Node):
    def __init__(self, action_block_data):
        #global unique_node_counter
        #node_name = f"grasp_service_client_node_{unique_node_counter}"
        node_name = f"grasp_service_client_node"
        #unique_node_counter += 1

        super().__init__(node_name)

        self.action_block_data = action_block_data

        self.ref_frame_name = "Ref_Frame"
        self.tool_frame = self.get_tool_frame()

        self.create_ref_frame_service_name = '/object_manager/create_ref_frame'
        self.move_tool_service_name = '/pm_moveit_server/move_tool_to_frame'
        self.change_parent_frame_service_name = '/object_manager/change_obj_parent_frame'

        self.create_ref_frame_client = self.create_client(CreateRefFrame, self.create_ref_frame_service_name)
        self.move_tool_client = self.create_client(MoveToolTcpTo, self.move_tool_service_name)
        self.change_parent_frame_client = self.create_client(ChangeParentFrame, self.change_parent_frame_service_name)

        self.rate_limit_delay = 1.0
        self.callback = None
        self.lock = threading.Lock()

    def set_callback(self, callback):
        self.callback = callback

    def send_request(self):
        if self.create_ref_frame_client.wait_for_service(1.0):
            self.create_ref_frame()
        else:
            self.get_logger().warn("CreateRefFrame service not available")

        if self.move_tool_client.wait_for_service(1.0):
            self.move_to_grasping_position()
        else:
            self.get_logger().warn("MoveToolTcpTo service not available")

        if self.change_parent_frame_client.wait_for_service(1.0):
            self.change_parent_frame("Test_Component")
        else:
            self.get_logger().warn("ChangeParentFrame service not available")

    def get_tool_frame(self):
        selected_tool = self.action_block_data["parameters"]["Tool"]
        if selected_tool == "Vacuum_Tool":
            return "PM_Robot_Vacuum_Tool"
        elif selected_tool == "Standard_Tool":
            return "PM_Robot_Tool_TCP"
        else:
            self.get_logger().error(f"Unsupported tool selected: {selected_tool}")
            return None 

    def create_ref_frame(self):
        request = self.create_request_for_create_ref_frame()
        future = self.create_ref_frame_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info("Reference frame created successfully")
        else:
            self.get_logger().error("Failed to create reference frame")

    def move_to_grasping_position(self):
        request = self.create_request_for_move_tool()
        future = self.move_tool_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info("Moved to grasping position successfully")
        else:
            self.get_logger().error("Failed to move to grasping position")

    def change_parent_frame(self, obj_name):
        request = self.create_request_for_change_parent_frame(obj_name)
        future = self.change_parent_frame_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info("Changed parent frame successfully")
        else:
            self.get_logger().error("Failed to change parent frame")

    def create_request_for_create_ref_frame(self):
        request = CreateRefFrame.Request()
        request.frame_name = self.ref_frame_name
        request.parent_frame = self.action_block_data["parameters"]["TargetComponent"]
        pose = geometry_msgs.msg.Pose()
        pose.position.x = -0.01755 #delta -0.0351
        pose.position.y = -0.010935 #delta -0.02186
        pose.position.z = 0.002359
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        request.pose = pose
        return request

    def create_request_for_move_tool(self):
        request = MoveToolTcpTo.Request()
        request.frame_name = self.ref_frame_name
        request.translation.x = 0.0
        request.translation.y = 0.0
        request.translation.z = 0.001
        request.exec_wait_for_user_input = False
        request.execute = True
        return request

    def create_request_for_change_parent_frame(self, obj_name):
        request = ChangeParentFrame.Request()
        request.obj_name = obj_name
        request.parent_frame = self.tool_frame
        return request

    def __del__(self):
        self.destroy_node()

def main(args=None):

    rclpy.init(args=args)

    action_block_data = {
        'action_class': 'Grasp',
        'parameters': {
            'TargetComponent': 'Test_Component',
            'Tool': 'Standard_Tool' 
        }
    }

    grasp_service_client = GraspServiceClient(action_block_data)

    try:
        rclpy.spin(grasp_service_client)
    finally:
        grasp_service_client.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
