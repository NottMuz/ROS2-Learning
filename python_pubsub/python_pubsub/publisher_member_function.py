# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#import the rosclient python library
import rclpy
#from the rosclient python.node library import the Node class
from rclpy.node import Node

#from the the std_msgs.msg module import the "String" message type
from std_msgs.msg import String

#creates a class, "MinimalPublisher", that is a subclass of class "Node"
class MinimalPublisher(Node):

    #defines the constructor "__init__" with the parameter being the
    #instance (object of the class) itself. We are essentially
    #initializing an object of this class
    def __init__(self):

        #like in java, "super" calls the constructor of the superclass "Node"
        #which allows it to repurpose the code for that constructor to
        #create a "minimal_publisher" node
        super().__init__('minimal_publisher')

        # ".publisher_" is an attribute inherited by the "Node" class which 
        #is how we are allowed to instantiate it and set/get ATTRIBUTES
        #like the "publisher_" attribute of the "Node" class

        #"create_publisher" is just a method/function that passes the 
        #message type, topic name, and queue size
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        #sets the timer period 
        timer_period = 0.5  # seconds

        #initializes the timer attribute of the instance, to have a period of .5 sec and to
        #call back to a provided function every period
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #initializes an instance variable that will track the amount of times the callback timer
        #is called 
        self.i = 0

    #defines the method with the parameter being the instance on which the method is called
    def timer_callback(self):
        
        #instantiates an object of the STRING message type from the imported package
        msg = String()

        #sets the data attribute of the msg to the following
        # "%d" : placeholder for an integer
        # "% self.i" : the interger for the place holder
        msg.data = 'Hello World: %d' % self.i

        #utilizing the publisher attribute of the current instance, publish the msg
        self.publisher_.publish(msg)

        #returns a logger object that has a method "info" that logs an informational msg
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1



#First the rclpy library is initialized, then the node is created, and then it “spins” 
#the node so its callbacks are called.

def main(args=None):    
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
