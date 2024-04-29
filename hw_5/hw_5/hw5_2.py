import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan as Scan
from geometry_msgs.msg import Twist
#from std_msgs.msg import String, Float64MultiArray
import math

class FollowInnerWall(Node):
    def __init__(self):
        super().__init__('turtle_follower')
        
        # set initial values for relevant distances
        self.side_distance = 999
        self.forward_distance = 999
        self.backward_distance = 999
        self.distance = 999
        
        ### 
        ### BONUS ###
        
        # declare parameters and initial values
        self.declare_parameter('distance_threshold', 0.5)
        self.declare_parameter('clockwise_turn', True)
        
        # set parameter values from launch file
        self.threshold = self.get_parameter('distance_threshold').value
        self.turn_cw = self.get_parameter('clockwise_turn').value
        ###
        ###
        
        # initial state
        self.state = 'searching_for_wall'
        
        # boolean for initial wall location logic
        self.wall_locating = True
        
        # initialization of publisher
        self.cmdvel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # initialization of main timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # initialization of scan subscription
        self.scan_subscriber = self.create_subscription(Scan, '/scan', self.get_scan_data, 10)
        
    # get relevant data from lidar scan
    def get_scan_data(self, data):
        # define specific angles to get data from
        distance_index = 0
        forward_index = 60 if self.turn_cw else 300
        backward_index = 120 if self.turn_cw else 240
        side_index = 90 if self.turn_cw else 270

        # set distances using angles
        self.distance = data.ranges[distance_index]
        self.side_distance = data.ranges[side_index]
        self.forward_distance = data.ranges[forward_index]
        self.backward_distance = data.ranges[backward_index]

    # main control loop
    def timer_callback(self):
        message = Twist()

        # initial and main state that either searches for or follows a wall
        if self.state == 'searching_for_wall':
            # if there is a wall within threshold, transition to initial rotation or corner state depending
            if self.distance < self.threshold:
                self.state = 'rotating_at_first_wall' if self.wall_locating else 'rotating_at_corner'
                # disable one-time boolean for wall locating
                self.wall_locating = False
                message.linear.x = 0.0
                message.angular.z = 0.0
            # otherwise, follow wall
            else:
                # adjust for distance if not keeping proper distance from and angle to wall
                if self.backward_distance > (2.0 / math.sqrt(3)) * (self.side_distance) or self.side_distance < self.threshold:
                    message.angular.z = -0.05 if self.turn_cw else 0.05
                elif self.backward_distance < (2.0 / math.sqrt(3)) * (self.side_distance):
                    message.angular.z = 0.05 if self.turn_cw else -0.05
                else:
                    message.angular.z = 0.0
                
                # set linear velocity
                message.linear.x = 0.5
        
        # state to perform initial rotation at wall
        elif self.state == 'rotating_at_first_wall':
            # if parallel to wall, transition to wall-following state
            if self.backward_distance <= self.forward_distance:
                self.state = 'searching_for_wall'
                message.angular.z = 0.0
            # otherwise, continue rotating
            else:
                message.angular.z = -0.2 if self.turn_cw else 0.2
                message.linear.x = -0.05
        
        # rotate at corner
        elif self.state == 'rotating_at_corner':
            # stop rotating and transition back to wall following once properly oriented
            if self.distance == float('inf') and self.backward_distance <= self.forward_distance:
                self.state = 'searching_for_wall'
                message.angular.z = 0.0
            # otherwise, continue rotating
            else:
                message.angular.z = -0.2 if self.turn_cw else 0.2
                message.linear.x = -0.05
        
        # log current state
        self.get_logger().info(self.state)
        
        # send commands to robot
        self.cmdvel_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    turtle_follower = FollowInnerWall()

    rclpy.spin(turtle_follower)

    turtle_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()