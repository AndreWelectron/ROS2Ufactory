import sys

from xarm_msgs.srv import MoveJoint, MoveCartesian, VacuumGripperCtrl, SetFloat32
import rclpy
from rclpy.node import Node
import threading



class PythonUFactoryClient(Node):

    def __init__(self):
        super().__init__('python_ufactory_client')
        self.cli = self.create_client(MoveJoint, 'ufactory/set_servo_angle')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveJoint.Request()

    def send_request(self, j1, j2, j3, j4, j5, j6):
        self.req.angles = (float(j1), float(j2), float(j3), float(j4), float(j5), float(j6))
        self.req.speed = 0.90
        self.req.mvtime = 0.0
        self.req.wait = False
        self.req.timeout = -1.0
        self.req.radius = -1.0
        self.req.relative = False
        self.req.acc = 10.0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class CartesianMoveUFactoryClient(Node):
    def __init__(self):
        super().__init__('cartesian_move_ufactory_client')
        self.cli = self.create_client(MoveCartesian, 'ufactory/set_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveCartesian.Request()
        self.running = True

    def send_request(self, x_pose, y_pose, z_pose):
            
            self.req.pose = (x_pose, y_pose, z_pose, 3.09, 0.03, 1.50)
            self.req.speed = 150.0
            self.req.mvtime = 0.0
            self.req.wait = False
            self.req.timeout = -1.0
            self.req.radius = -1.0
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)

class VacuumGripperUFactoryClient(Node):
    def __init__(self):
        super().__init__('vacuum_gripper_ufactory_client')
        self.cli = self.create_client(VacuumGripperCtrl, 'ufactory/set_vacuum_gripper')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = VacuumGripperCtrl.Request()
        self.running = True

    def send_request(self, is_active):
            
            self.req.on = is_active
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)

class PauseTimeUFactoryClient(Node):
    def __init__(self):
        super().__init__('pause_time_ufactory_client')
        self.cli = self.create_client(SetFloat32, 'ufactory/set_pause_time')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetFloat32.Request()
        self.running = True

    def send_request(self, wait_time):
            
            self.req.data = wait_time
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)

def main(args=None):
    rclpy.init(args=args)

    ufactory_control_move_joint = PythonUFactoryClient()
    ufactory_control_move_cartesian = CartesianMoveUFactoryClient()
    ufactory_control_vacuum_gripper = VacuumGripperUFactoryClient()
    ufactory_control_pause_time = PauseTimeUFactoryClient()

    for j in range(2):
        for i in range(3):
            response1 = ufactory_control_move_joint.send_request(-1.57, 0.31, 1.04, 3.23, -0.70, 0)
            ufactory_control_move_joint.get_logger().info('The Robot will move to chips matrix...')

            x_matrix= -17 + (100*i)
            y_matrix= -309 + (90*j)
            z_matrix= 67

            response2 = ufactory_control_move_cartesian.send_request(x_matrix, y_matrix, z_matrix)
            ufactory_control_move_cartesian.get_logger().info('The Robot will move to the (i,j) chip...')

            response_gripper = ufactory_control_vacuum_gripper.send_request(True)
            ufactory_control_vacuum_gripper.get_logger().info('The Robot will activate the vacuum gripper...')

            response_pause_time = ufactory_control_pause_time.send_request(0.5)
            ufactory_control_pause_time.get_logger().info('The Robot will pause for 0.5 seconds...')

            
            response3 = ufactory_control_move_joint.send_request(0, 0.17, 1.11, 3.10, -0.85, 0)
            ufactory_control_move_joint.get_logger().info('The Robot will move above of stack of cards joints...')

            x_stack=278
            y_stack=12
            z_stack=80

            response_4 = ufactory_control_move_cartesian.send_request(x_stack, y_stack, z_stack)
            ufactory_control_move_cartesian.get_logger().info('The Robot will move to stack of cards...')

            response_gripper = ufactory_control_vacuum_gripper.send_request(False)
            ufactory_control_vacuum_gripper.get_logger().info('The Robot will deactivate the vacuum gripper...')

            response_pause_time = ufactory_control_pause_time.send_request(0.5)
            ufactory_control_pause_time.get_logger().info('The Robot will pause for 0.5 seconds...')

            response5 = ufactory_control_move_joint.send_request(0, 0.17, 1.11, 3.10, -0.85, 0)
            ufactory_control_move_joint.get_logger().info('The Robot will move above of stack of cards joints...')


    ufactory_control_move_joint.destroy_node()
    ufactory_control_move_cartesian.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
