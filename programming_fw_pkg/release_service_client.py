import rclpy
from rclpy.node import Node
from spawn_object_interfaces.srv import ChangeParentFrame
import threading

class ReleaseServiceClient(Node):
    def __init__(self, action_block_data):
        node_name = f"release_service_client_node"
        super().__init__(node_name)

        self.action_block_data = action_block_data
        # self.tool_frame = self.get_tool_frame()
        self.target_frame = self.get_target_frame()

        self.change_parent_frame_service_name = '/object_manager/change_obj_parent_frame'
        self.change_parent_frame_client = self.create_client(ChangeParentFrame, self.change_parent_frame_service_name)

        self.rate_limit_delay = 1.0
        self.callback = None
        self.lock = threading.Lock()

    def set_callback(self, callback):
        self.callback = callback

    def send_request(self):
        if self.change_parent_frame_client.wait_for_service(1.0):
            self.change_parent_frame("Test_Component")
        else:
            self.get_logger().warn("ChangeParentFrame service not available")

    def change_parent_frame(self, obj_name):
        request = self.create_request_for_change_parent_frame(obj_name)
        future = self.change_parent_frame_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.handle_response(future, request)

    def create_request_for_change_parent_frame(self, obj_name):
        request = ChangeParentFrame.Request()
        request.obj_name = obj_name
        request.parent_frame = self.target_frame
        return request
    
    def get_target_frame(self):
        target = self.action_block_data["parameters"]["Target"]
        if target == "Gonio_Left_Part_Origin":
            return "Gonio_Left_Part_Origin"
        else:
            self.get_logger().error(f"Unsupported target selected: {target}")
            return None 

    def handle_response(self, future, request):
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info(f"Changed parent frame successfully for {request.obj_name}")
        else:
            self.get_logger().error(f"Failed to change parent frame for {request.obj_name}")

def main(args=None):

    rclpy.init(args=args)

    action_block_data = {
        'action_class': 'Release',
        'parameters': {
            'Component': 'Test_Component',
            'Target': 'Gonio_Left_Part_Origin' 
        }
    }

    grasp_service_client = ReleaseServiceClient(action_block_data)

    try:
        rclpy.spin(grasp_service_client)
    finally:
        grasp_service_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
