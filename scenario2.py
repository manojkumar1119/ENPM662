#!/usr/bin/env python3
#importing librries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt

#class for scenario2
class scenario2(Node):
    def __init__(self):
        super().__init__('scenario2')
        #creating a publisher 
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #call back function
        self.timer = self.create_timer(0.1, self.update_position)
        self.start_time = None
        #setting the distance , and acleration parameters
        self.linear_accel = 0.1  # Acceleration 
        self.linear_decel = -0.1  # Deceleration
        self.total_distance = 1.0  
        self.max_linear_speed = 0.15 
        #plotting lists for appending
        self.displacement_lst = []
        self.time_lst = []         

        # Distance calculations
        self.dist_accel = (self.max_linear_speed ** 2) / (2 * self.linear_accel)
        self.dist_decel = (self.max_linear_speed ** 2) / (2 * -self.linear_decel)
        # Calculate vel distances
        self.constant_speed_dist = max(0, self.total_distance - (self.dist_accel + self.dist_decel))
        # Initial state setting up
        self.current_speed = 0.0
        self.position = 0.0
        self.state = 'accelerating'        
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    #function for publishing 
    def send_command(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = float(self.current_speed)
        self.publisher.publish(cmd_msg)

    #function timer callback
    def update_position(self):
        elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        self.position += self.current_speed * 0.1  # Update position
        # Store position and time for plot
        self.displacement_lst.append(self.position)
        self.time_lst.append(elapsed_time)

        # State updates for accelertion
        if self.state == 'accelerating':
            if self.current_speed < self.max_linear_speed:
                self.current_speed += self.linear_accel * 0.1
                if self.current_speed >= self.max_linear_speed:
                    self.current_speed = self.max_linear_speed
                    self.state = 'constant'        
        #deceleration
        elif self.state == 'decelerating':
            if self.current_speed > 0:
                self.current_speed += self.linear_decel * 0.1
                if self.current_speed < 0:
                    self.current_speed = 0  # Stop the turtlebot
                    self.timer.cancel()  # Stop the timer
                    self.plot_path()  # Plotting
        #constant speed
        elif self.state == 'constant':
            if self.position < (self.dist_accel + self.constant_speed_dist):
                self.current_speed = self.max_linear_speed
            else:
                self.state = 'decelerating'

        self.send_command()

    #plotting the path
    def plot_path(self):
        plt.plot(self.time_lst, self.displacement_lst)
        plt.xlabel('Time in seconds')
        plt.ylabel('Position in meters')
        plt.title('Path')
        plt.show()

#main function
def main(args=None):
    rclpy.init(args=args)
    #object creation
    bot_controller = scenario2()
    rclpy.spin(bot_controller)
    #destroy the node
    bot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()