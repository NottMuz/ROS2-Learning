import rclpy
import rclpy.node

#"MinimalParam" inherits from the rosclient.node Node class
class MinimalParam(rclpy.node.Node):

    #constructor for the MinimalParam Instance
    def __init__(self):
        
        #parameter descriptor for seeing paramter info via cmd line etc
        from rcl_interfaces.msg import ParameterDescriptor
        my_parameter_descriptor = ParameterDescriptor(description='This parameter is mine!')

        #call the superclass constructor to create the node with the specified name
        super().__init__('minimal_param_node')

        #declare the parameter and its value, as well as its description
        self.declare_parameter('my_parameter', 'world', my_parameter_descriptor)

        #create the timer to call the call back function of the instance every 1 second
        self.timer = self.create_timer(1, self.timer_callback)
    

    #collects the parameter information then displays it
    def timer_callback(self):

        #gets the parameter value from the parameter 
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        #log and display the parameter information
        self.get_logger().info('Hello %s!' % my_param)

        #essentially a parameter constructor to create a "new parameter",even though it has the same value
            # as before it is used to change back to default if the parameter is changed externally
        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )

        #creates a list containing the parameter variable constructed previously
        all_new_parameters = [my_new_param]

        #sets all the parameters to the parameters in the list
        self.set_parameters(all_new_parameters)

#standard main function to initialize and start the node
def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()