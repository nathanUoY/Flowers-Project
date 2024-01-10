import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class RobotController(Node):

    def __init__(self, robot_name, robot_namespace):
        super().__init__('robot_controller_' + robot_name, namespace = robot_namespace)

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # Linear and angular velocities for robot movement
        self.linear_speed = 0.2
        self.angular_speed = 0.1

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Color detection parameters for RED
        self.lower_red = np.array([0, 0, 100])
        self.upper_red = np.array([100, 100, 255])

        self.cv_image = None

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def control_loop(self):
        if self.cv_image is not None:
            # Perform color detection (example: detect red color)
            mask = cv2.inRange(self.cv_image, self.lower_red, self.upper_red)
            result_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Assuming the robot should move towards the center of the detected object
                moments = cv2.moments(contours[0])
                if moments['m00'] != 0:
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])
                    self.move_towards_point(cx, cy)

            msg = Twist()
            msg.linear.x = self.linear_speed
            self.cmd_vel_publisher.publish(msg)

            # Display the results (optional)
        #    cv2.imshow(f'Color Detection - {self.get_namespace()}', result_image)
        #    cv2.waitKey(1)

    def image_callback(self,msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
            return

        # Perform color detection (example: detect red color)
        mask = cv2.inRange(self.cv_image, self.lower_red, self.upper_red)
        result_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Assuming the robot should move towards the center of the detected object
            moments = cv2.moments(contours[0])
            if moments['m00'] != 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                self.move_towards_point(cx, cy)

        # Display the results (optional)
        cv2.imshow('Color Detection', result_image)
        cv2.waitKey(1)

    def move_towards_point(self, cx, cy):
        # Calculate the error between the center of the image and the center of the object
        error = cx - (self.cv_image.shape[1] // 2)

        # Adjust the angular velocity based on the error
        self.angular_speed = -0.01 * error

        # Publish the movement command
        self.publish_movement_command()

    def publish_movement_command(self):
        # Publish a movement command (e.g., Twist message)
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = self.angular_speed

        # Publish the Twist message
        self.velocity_publisher.publish(twist_msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    #add delay before starting node
    time.sleep(1.0)

    #node = RobotController()
    robot1 = RobotController(robot_name='robot1')
    robot2 = RobotController(robot_name='robot2')


    try:
        rclpy.spin(robot1)
        rclpy.spin(robot2)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        #node.destroy_node()
        robot1.destroy_node()
        robot2.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()