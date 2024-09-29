#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time as time_clock

#Creating a class for the scenario1 
class scenario1_bot(Node):
    #init function
    def __init__(self):
        #node name
        super().__init__("scenario1")
        #publisher creation
        self.pub = self.create_publisher(Twist,'/cmd_vel',8)
        self.rate = self.create_rate(10)
        #defining the lists for time and displacement to plot
        self.time = []
        self.displacement = []
        
    #Function to make the turtlebot to move in staright line
    def turtlebot_move(self, time, dist):
        #finding the velocity
        velocity = dist / time
        #setting the msg
        vel_msg = Twist()
        vel_msg.linear.x = velocity
        #getting the time for plotting         
        strt_time = time_clock.time()

        #looping over till time 10 s is reached        
        while ((time_clock.time() - strt_time) < time):
            self.get_logger().info(f"Velocity:{velocity}")
            #publish the velocity
            self.pub.publish(vel_msg)
            curr_time = (time_clock.time()-strt_time )
            self.get_logger().info(f"Time:{curr_time}")
            #get the displacement = vel * time
            self.displacement.append(velocity * (curr_time))
            self.get_logger().info(f"Displacement:{velocity* (curr_time)}")
            #appending time to list
            self.time.append(curr_time)
            
        #make the turtlebot to stop
        vel_msg.linear.x = 0.0
        self.pub.publish(vel_msg) 
        
def main(args=None):
    rclpy.init(args=args)
    #creating an object
    controller = scenario1_bot()
    #calling function to move the turtlebot
    #10 seconds , 1 meter
    controller.turtlebot_move(10.0,1.0)
    #plotting the path
    plt.plot(controller.time, controller.displacement)
    plt.xlabel('Time in Seconds')
    plt.ylabel('Position in meters')
    plt.title('Path')
    plt.show()
    #shutdown
    rclpy.shutdown()

#Main function 
if __name__ == '__main__':
    main()
