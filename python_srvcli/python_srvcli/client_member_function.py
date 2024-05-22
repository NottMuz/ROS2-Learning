import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

#creates the client class
class MinimalClientAsync(Node):

    #constructor for the client class objects
    def __init__(self):
        
        #constructs the client node using the superclass constructor
        super().__init__('minimal_client_async')

        #initializes the client attribute of the node/object and defines the client type, 
            #and the service name
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        #checks if there is a service that matches the type of the client is available every second
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        #sets the request attribute of the client object to follow the message type defined by
            #"AddTwoInts.Request()"
        self.req = AddTwoInts.Request()

    #function definition for populating the request variables, sending the request, and recieving
        #the result
    def send_request(self, a, b):

        #take the command line arguments for the req and set the "a" and "b" members accordingly
        self.req.a = a
        self.req.b = b
        
        #"self.future" : holds the future (reply/service response) object/variable
        #"self.cli.call_async(self.req)" : asynchronously call the service and provide the request
        self.future = self.cli.call_async(self.req)
        
        #spins until the response is recieved and returns the result(reply)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    
    #initialize the rosclient python library
    rclpy.init(args=args)

    #create an instance of the MinimalClientAsync Class by calling the constructor
    minimal_client = MinimalClientAsync()

    #turn the cmd line arguments into intergers and pass them as parameters to the 
        #"send_request" function definition
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    #log and display the information in the required format
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()