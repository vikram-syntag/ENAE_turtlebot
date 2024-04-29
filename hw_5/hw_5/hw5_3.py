import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan as Scan
from geometry_msgs.msg import Twist
#from std_msgs.msg import String, Float64MultiArray
import math

class FollowOuterWall(Node):
    def __init__(self):
        super().__init__('turtle_follower')

        # distance to and angle from wall
        self.euclidean_distance = 999
        self.euclidean_angle = 0
        
        # distance threshold
        self.threshold = 0.5
        
        # initial state
        self.state = "searching_for_wall"

        # initialization of publisher
        self.cmdvel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # initialization of main timer
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # initialization of scan subscription
        self.scan_subscriber = self.create_subscription(Scan, '/scan', self.get_scan_data, 10)

    # get relevant data from lidar scan
    def get_scan_data(self, data):
        self.euclidean_distance = min(data.ranges)
        self.euclidean_angle = data.ranges.index(self.euclidean_distance)

    # main control loop
    def timer_callback(self):
        message = Twist()

        # initial state where robot finds wall
        if self.state == 'searching_for_wall':
            # once within range defined by threshold, stop moving and transition to rotating state
            if self.euclidean_distance < self.threshold:
                self.state = 'rotating_at_first_wall'
                message.linear.x = 0.0
                message.angular.z = 0.0
            # otherwise, keep moving forward
            else:
                message.angular.z = 0.0
                message.linear.x = 0.5
        
        # secondary state where robot turns left at wall so it can turn clockwise for the rest
        elif self.state == 'rotating_at_first_wall':
            # stop rotating once roughly parallel with wall and transition to wall-following state
            if self.euclidean_angle > 268 and self.euclidean_angle < 272:
                self.state = 'following_wall'
                message.angular.z = 0.0
                message.linear.x = 0.0
            # otherwise, keep rotating
            else:
                message.angular.z = 0.1
                message.linear.x = -0.018
        
        # main state where robot follows wall and turns at corners
        elif self.state == 'following_wall':
            # value by which to adjust angular velocity to maintain threshold distance from wall
            adjustment = 1.2 * (self.threshold - self.euclidean_distance)
            
            # set angular velocity based on current angle and distance relative to wall/corner
            message.angular.z = adjustment -(270 - self.euclidean_angle) * (2 * math.pi / 360)
            
            # set linear velocity
            message.linear.x = 0.5
        
        # log current state
        self.get_logger().info(self.state)
        
        # send commands to robot
        self.cmdvel_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    turtle_follower = FollowOuterWall()

    rclpy.spin(turtle_follower)

    turtle_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()