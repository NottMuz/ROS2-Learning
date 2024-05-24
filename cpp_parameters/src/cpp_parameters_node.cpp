#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{

//constructor for the class
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    //creates a parameter description
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is mine!";

    //creates a parameter called "my_...", with a default value of "world"
      //the parameter type is INFERRED, so in this case it would be of type String
      // and apply a description to th
    this->declare_parameter("my_parameter", "world", param_desc);

    //creates a timer that repeatedly calls the binded callback function every 1s
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback()
  {
    //gets the parameter from the node
    std::string my_param = this->get_parameter("my_parameter").as_string();

    //logs and displays the parameter
    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    //creates a vector of type parameter, that holds a parameter called "my_...", with a value "goop"
    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "goop")};

    //sets all the previously declared parameters to the parameters in the vector, already the same but ok
    this->set_parameters(all_new_parameters);
  }

private:
//allocates a smart pointer to the timer_ object/variable for memory management
  rclcpp::TimerBase::SharedPtr timer_;
};

//main function for running the node
int main(int argc, char ** argv)
{
  
  //initializes the node based off of the cmd line argument and passed values
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}