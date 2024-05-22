#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{  

  //initializes the rosclient C++ library
  rclcpp::init(argc, argv);
 
  //checks to see if the ammount of cmd line arguments is correct (expects 3)
  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }
  
  //assigns a smart pointer to the client node that is being initialized
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");

  //setup the client nodes "client" attribute
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  //creates a shared pointer to a request object from the addtwoints service
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

  // "->a" : access member "a" of the object
  // "argv" : array of C-style strings that rep. cmd. line arg.
  // "atoll" : (arg. to long long) converts the argument to a long long interger value
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  //while the client cannot find the service after a 1 second period
  while (!client->wait_for_service(1s)) {

    //check if ROS context is NOT(!) alive (i.e there is shutdown signal or user interrupt)
    if (!rclcpp::ok()) 
    {
      //Terminate the node
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    //loop back to check for another second if service was not found and check for it again
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  //contains the request object to be sent to the server
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}