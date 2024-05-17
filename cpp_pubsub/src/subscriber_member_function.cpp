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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//use the placeholders namespace from the C++ standard library
  //contains _1, _2, _3 ...
//it's use lies with the use of the binding function (provided by std::bind), it provides a placeholder
  //for a function(s) paramters that are not binded, see its application below in the MinimalSubscriber Node 
using std::placeholders::_1;

//Create a public class "Minimal Subscriber" that inherits from the rosclient class "Node"
class MinimalSubscriber : public rclcpp::Node
{
  //public constructor for the class
public:
  MinimalSubscriber()
  
  //names the node
  : Node("minimal_subscriber")
  {
    //-------------------------------------------------------------------------------------------------------
    //set's up our subscriber variable and its type

      //"this->" is a pointer to this classes's (MinimalSubsciber) instance, allowing for access to all
        //variables and functions, in order to utilize them to create a subscriber

      //"create_subscription<std_msgs::msg::String>" is a TEMPLATE FUNCTION CALL, essentially allowing it
        //to operate with various data types, which makes sense in terms of messages and how their are 
        //multimple message types
    subscription_ = this->create_subscription<std_msgs::msg::String>(

      //in these brackets we establish our parameters for the topic 
        //"topic" : topic name we are subscribing to
        //"10" : the queue size for the incoming messages

      //this one is a bit tricky so it deserves seperate attention
        // "std::bind(&MinimalSubscriber::topic_callback, this, _1)": essentially we are binding a 
          //pointer to the "topic_callback" function of 'THIS' classes's instance and SINCE THE 
          //PARAMETER FOR THE FUNCTION IS A MSG OF TYPE STRING THE PLACEHOLDER (_1) IS FOR THAT MSG
          //TO BE SENT TO THE FUNCTION!!!!!!
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    //-------------------------------------------------------------------------------------------------------
  }

private:
//void return type as we are just executing the function
//paramter is a msg that is a shared pointer to type String
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
            // Inside the function:
        // Log a message using the ROS 2 logging mechanism
        // "I heard: '%s'" is the format string
        // msg->data.c_str() retrieves the string data from the message and converts it to a C-style string
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
