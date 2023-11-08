import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

#TODO: Import the message types that you need for your publishers and subscribers here:
from sensor_msgs.msg import LaserScan

class BraitenbergController(Node):
    def __init__(self):
        super().__init__('braitenberg_controller')


        # Robot TB3_0
        self.lidar_subscriber_tb3_0 = self.create_subscription(
            LaserScan,
            '/tb3_0/scan',
            self.clbk_laser_tb3_0,
            10
        )

        self.vel_publisher_tb3_0 = self.create_publisher(
            Twist,
            '/tb3_0/cmd_vel',
            10
        )

        self.lidar_left_front_tb3_0 = 100
        self.lidar_right_front_tb3_0 = 100

        # Robot TB3_1
        self.lidar_subscriber_tb3_1 = self.create_subscription(
            LaserScan,
            '/tb3_1/scan',
            self.clbk_laser_tb3_1,
            10
        )

        self.vel_publisher_tb3_1 = self.create_publisher(
            Twist,
            '/tb3_1/cmd_vel',
            10
        )

        self.lidar_left_front_tb3_1 = 100
        self.lidar_right_front_tb3_1 = 100

        # Timer setup
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    # Callback function for the TB3_0 Lidar topic /scan
    def clbk_laser_tb3_0(self, msg):
        self.lidar_left_front_tb3_0 = msg.ranges[12]
        self.lidar_right_front_tb3_0 = msg.ranges[348]

    # Callback function for the TB3_1 Lidar topic /scan
    def clbk_laser_tb3_1(self, msg):
        self.lidar_left_front_tb3_1 = msg.ranges[12]
        self.lidar_right_front_tb3_1 = msg.ranges[348]

    def timer_callback(self):
        # Process for TB3_0
        vel_msg_tb3_0 = self.calculate_velocity(self.lidar_left_front_tb3_0, self.lidar_right_front_tb3_0)
        self.vel_publisher_tb3_0.publish(vel_msg_tb3_0)

        # Process for TB3_1
        vel_msg_tb3_1 = self.calculate_velocity(self.lidar_left_front_tb3_1, self.lidar_right_front_tb3_1)
        self.vel_publisher_tb3_1.publish(vel_msg_tb3_1)

    def calculate_velocity(self, left_dist, right_dist):
        # Create the velocity message
        vel_msg = Twist()
        # If obstacles are very close, reverse and turn
        if left_dist < 0.5 and right_dist < 0.5:
            vel_msg.linear.x = -0.2  # Reverse a bit
            vel_msg.angular.z = 0.5  # And turn
        elif left_dist < 0.5:
            vel_msg.linear.x = -0.2
            vel_msg.angular.z = -0.5  # Turn right
        elif right_dist < 0.5:
            vel_msg.linear.x = -0.2
            vel_msg.angular.z = 0.5   # Turn left
        # If obstacles are detected but not too close, turn without reversing
        elif left_dist < 1.0:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.5  # Turn right
        elif right_dist < 1.0:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5   # Turn left
        else:
        # If no obstacles are close, keep moving forward with no angular change
            vel_msg.linear.x = 0.7
            vel_msg.angular.z = 0.0

        return vel_msg


def main(args=None):
    rclpy.init(args=args)

    controller = BraitenbergController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()