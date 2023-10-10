import rclpy
from rclpy.node import Node
from pm_moveit_interfaces.srv import MoveCam1TcpTo, MoveLaserTcpTo, MoveToolTcpTo
import geometry_msgs
import threading
import time

class MoveRelativeServiceClient(Node):
    def __init__(self, action_block_data):
        super().__init__('move_relative_service_client')
        self.action_block_data = action_block_data
        print("Contents of action_block_data:", self.action_block_data)

        self.move_group = self.action_block_data["parameters"]["MoveGroup"]

        if self.move_group == "Cam1":
            self.service_name = 'pm_moveit_server/move_cam1_to_frame'
            self.service_client = self.create_client(MoveCam1TcpTo, self.service_name)
        elif self.move_group == "Tool":
            self.service_name = 'pm_moveit_server/move_tool_to_frame'
            self.service_client = self.create_client(MoveToolTcpTo, self.service_name)
        elif self.move_group == "Laser":
            self.service_name = 'pm_moveit_server/move_laser_to_frame'
            self.service_client = self.create_client(MoveLaserTcpTo, self.service_name)
        else:
            self.get_logger().error(f"Unsupported move group: {self.move_group}")
            return

        self.rate_limit_delay = 1.0
        self.callback = None
        self.lock = threading.Lock()

    def set_callback(self, callback):
        self.callback = callback

    def send_request(self):
        if self.service_client.wait_for_service(1.0):
            request = self.create_request()
            self.set_callback(self.handle_response)

            with self.lock:
                self.future = self.service_client.call_async(request)
                self.future.add_done_callback(self.handle_response)
            
                time.sleep(self.rate_limit_delay)
        else:
            self.get_logger().warn(f'Service {self.service_name} not available')

    def create_request(self):
        request = None
        if self.move_group == "Cam1":
            request = MoveCam1TcpTo.Request()
            request.translation = self.parse_translation(self.action_block_data["parameters"]["Offset (x, y, z)"])
            request.rotation = self.parse_rotation(self.action_block_data["parameters"]["Rotation (x, y, z, w)"])
            request.exec_wait_for_user_input = self.action_block_data["parameters"]["Wait for User Input"]
            request.execute = True 
        elif self.move_group == "Tool":
            request = MoveToolTcpTo.Request()
            request.translation = self.parse_translation(self.action_block_data["parameters"]["Offset (x, y, z)"])
            request.rotation = self.parse_rotation(self.action_block_data["parameters"]["Rotation (x, y, z, w)"])
            request.exec_wait_for_user_input = self.action_block_data["parameters"]["Wait for User Input"]
            request.execute = True  
        elif self.move_group == "Laser":
            request = MoveLaserTcpTo.Request()
            request.translation = self.parse_translation(self.action_block_data["parameters"]["Offset (x, y, z)"])
            request.rotation = self.parse_rotation(self.action_block_data["parameters"]["Rotation (x, y, z, w)"])
            request.exec_wait_for_user_input = self.action_block_data["parameters"]["Wait for User Input"]
            request.execute = True  
        return request
    
    def parse_translation(self, translation_str):
        translation_values = translation_str.split(', ')
        translation = geometry_msgs.msg.Vector3()
        translation.x = float(translation_values[0])
        translation.y = float(translation_values[1])
        translation.z = float(translation_values[2])
        return translation
    
    def parse_rotation(self, rotation_str):
        rotation_values = rotation_str.split(', ')
        rotation = geometry_msgs.msg.Quaternion()
        rotation.x = float(rotation_values[0])
        rotation.y = float(rotation_values[1])
        rotation.z = float(rotation_values[2])
        rotation.w = float(rotation_values[3])
        return rotation

    def handle_response(self, future, request):
        try:
            response = future.result()
            self.get_logger().info(f"Response Object: {response}")
            if response.success:
                self.get_logger().info(f"Service call succeeded for {self.move_group}")
            else:
                self.get_logger().warn(f"Service call failed for {self.move_group}")
        except Exception as e:
            self.get_logger().error(f'Service call failed for {self.move_group}: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    
    rclpy.init(args=args)

    action_block_data = {
        'action_class': 'MoveRelative',
        'parameters': {
            'MoveGroup': 'Tool',
            'Offset (x, y, z)': '0.0, 0.0, 0.0',
            'Rotation (x, y, z, w)': '0.0, 0.0, 0.0, 1.0',
            'Wait for User Input': False
        }
    }

    move_to_frame_client = MoveRelativeServiceClient(action_block_data)
    
    try:
        rclpy.spin(move_to_frame_client)
    finally:
        move_to_frame_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()