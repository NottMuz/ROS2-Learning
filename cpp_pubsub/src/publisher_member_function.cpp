// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



//-----------------------------------------------------------------------
/*
This section standard 'imports' header files that the package uses,
as well as common peices of the ROS 2 system, and finally the built in 
message type we will use
*/



//This header file provides classes and functions for handling time-related operations. 
//It includes facilities for measuring time durations and points in time, 
//as well as clocks to measure time intervals.
#include <chrono>

/*This header file provides templates and classes for working with function objects and function pointers. 
It includes various function objects like std::function, which can store any callable object 
(function pointer, lambda, etc.), as well as utilities for composing functions and function adaptors.*/
#include <functional>

/*This header file provides classes and functions for managing dynamic memory allocation and smart pointers. 
It includes the std::shared_ptr, std::unique_ptr, and std::weak_ptr smart pointer classes, as well as functions 
for dynamic memory allocation (e.g., std::make_shared, std::make_unique).*/
#include <memory>

/*This header file provides classes and functions for working with strings in C++. It includes the std::string 
class for representing and manipulating sequences of characters, as well as functions for string manipulation and conversion.*/
#include <string>

//Common peices of the ROS2 system
#include "rclcpp/rclcpp.hpp"

//built in message/message type we will use
#include "std_msgs/msg/string.hpp"  

//-----------------------------------------------------------------------


//"using namespcae" : allows us to bring all the symbols (functions, classes etc) of a namescpace into out current scope, 
//allowing for access to them withough prefixing the namespace
// "std::chrono_literals" : nested namespace of std::chrono, contains user defined literals for representing durations and time points
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
// inherits from the public class "Node" in the "rclcpp" namespace
class MinimalPublisher : public rclcpp::Node
{

//specifies class/constructor is public
public:

  //constructor for the class
  MinimalPublisher()

  //names the node to be "minimal_publisher" and initializes count to zero
  : Node("minimal_publisher"), count_(0)
  {
    //initializes the "Publisher" with a [string] message type, identifies it is publishing to "topic", max queue 
    //size is 10, meaning a maximum of 10 messages can be queued up for sending 
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    //"timer_" is initialized (variable of rclcpp::TimerBase::SharedPtr)
    //calls the "timer_callback" function twice a second
    timer_ = this->create_wall_timer( 500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

//private function accesible only in the class
private:

//void type function as no variable is needed for operation
  void timer_callback()
  {
    //"auto" : automatic type identifier
    // instantiates "message" variable as type string
    auto message = std_msgs::msg::String();

    //sets the data field of "message" to "Hello, world!" as well as concates that string with an increasing count variable
    message.data = "Hello, world! " + std::to_string(count_++);

    //logs a message using the logger associated with the node. it indicates that a message is being published (ensures published
    // messages are printed to the console)
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    //pushes "message" to be published via the "publisher_" variable we declared
    publisher_->publish(message);
  }

  //memory management?, do not fully understand yet
  // Declaration of the timer, publisher, and counter fields
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

//standard main function and main function parameters
//node actually executes here
int main(int argc, char * argv[])
{
  //initializes the ros2 client library, must be called before any of its other functions etc. are used
  rclcpp::init(argc, argv);

  //starts processing the data from the node "MakePublisher" (including callbacks from the timer)
  //"make_shared" : makes a new instance of the node using a "SharedPtr"
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  //called after spin exits, node is no longer needed and can shutdown
  rclcpp::shutdown();
  return 0;
}
