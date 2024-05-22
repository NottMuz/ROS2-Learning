from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node



#create the "MinimalService" class that inherits from the node class
class MinimalService(Node):

    #initializes a service object
    def __init__(self):

        #call the superclass constructor
        super().__init__('minimal_service')
        
        #set the srv msg type to "AddTwoInts", set the name to "add_t...", and identify the 
            #callback function when a request is recieved
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)



    #the definition of the service callback takes the server object, request and responses as
        # parameters and returns the response after computation
    def add_two_ints_callback(self, request, response):
        
        #access the "sum" attribute of the response and set it equal to the sum of the "a" and
            #"b" attributes of the REQUEST
        response.sum = request.a + request.b

        #log and desiplay the incoming request
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


#main class initializes the ROS 2 Python client library, instantiates the MinimalService 
    #class to create the service node and spins the node to handle callbacks.
def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()