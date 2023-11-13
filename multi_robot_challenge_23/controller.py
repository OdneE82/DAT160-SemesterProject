from multi_robot_challenge_23.go_to_point import GoToPointClass
from multi_robot_challenge_23.robot import RobotClass
from multi_robot_challenge_23.wall_follower import WallFollowerClass
from multi_robot_challenge_23.bug2 import bug2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.logging import get_logger

#TODO: Import the message types that you need for your publishers and subscribers here:
from sensor_msgs.msg import LaserScan

class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')

        self.robot_namespaces = ['tb3_0', 'tb3_1']

        self.robots = {}
        for namespace in self.robot_namespaces:
            self._logger.info(f'Initializing classes for {namespace}...')
            self.robots[namespace] = {
                'wall_follower': WallFollowerClass(namespace),
                'go_to_point': GoToPointClass(namespace),
                'bug2': bug2(namespace),
                'robot': RobotClass(namespace)            
            }

            self._logger.info('All classes initialized successfully for both robots.')


        """
        self.logger = get_logger('master_controller')

        self.logger.info('Initializing WallFollowerClass...')
        self.wall_follower = WallFollowerClass('/tb3_0')

        self.logger.info('Initializing GoToPointClass...')
        self.go_to_point = GoToPointClass('/tb3_0')

        self.logger.info('Initializing bug2...')
        self.bug2 = bug2('/tb3_0')

        self.logger.info('Initializing RobotClass...')
        self.robot = RobotClass('/tb3_0')

        self.logger.info('All classes initialized successfully.')

        robot_namespace_0 = 'tb3_0'
        self.wall_follower = WallFollowerClass(robot_namespace_0)
        self.go_to_point = GoToPointClass(robot_namespace_0)
        self.bug2 = bug2(robot_namespace_0)
        self.robot = RobotClass(robot_namespace_0)

        robot_namespace_1 = 'tb3_1'
        self.wall_follower = WallFollowerClass('wall_follower_node_1' + robot_namespace_1)
        self.go_to_point = GoToPointClass(robot_namespace_1)
        self.bug2 = bug2(robot_namespace_1)
        self.robot = RobotClass(robot_namespace_1)
        """




def main(args=None):
    rclpy.init(args=args)
    controller = MasterController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()