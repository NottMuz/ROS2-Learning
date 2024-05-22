//include the header file to work with the rosclient C++ library
#include "rclcpp/rclcpp.hpp"

//include the header file to use the "AddTwoInts" service message from the "exa..." package
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

//-----------------------------------------------------------------------------------------------------------

// - defining the "add" function that the service will provide

// - takes two parameters, the request and response of the service. Remember that <.....> is a TEMPLATE FUNCTION 
  // CALL, meaning it can accept multiple data types

// - "<example_interfaces::srv::AddTwoInts::Request>" : Identifies what object the smart pointer (shared_ptr) will
  //point to, in this case it is a Request object from the addtwoints srv msg

// - "request" is just the name of the obect/variable being declared
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  /*Before continuing, understand that the data structure of the addtwoints srv msg is:
      //int64 a
      //int64 b
      //---
      //int64 sum
  */

  // "response->sum" : Access the "sum" member/attribute of the 'response' object
  // "request->a"/b : Access the "a" and "b" attributes of the 'REQUEST' object
  response->sum = request->a + request->b;

  //log and show on the shell/terminal
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

//-----------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{

  //initializes the ROS2 C++ client library
  rclcpp::init(argc, argv);

  //creates the node, "add_two_ints_server" and assigns a pointer to this node object
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  //Creates a pointer to the service object
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
  //from this instance of the service node, call the "create_service" function to create a service 
    //from the template method "example_interfaces::srv::AddTwoInts"
  //the name of the node is "add_..." and the there is a pointer to the function "add" which will be called
    //whenever a request is recieved
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  //logging & prints the log
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  //keeps then node running
  rclcpp::spin(node);

  //terminates the node
  rclcpp::shutdown();
}
