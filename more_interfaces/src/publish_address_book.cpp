#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

//includes the header file for our msg type
#include "more_interfaces/msg/address_book.hpp"

using namespace std::chrono_literals;

//create a publisher class that inherits from the node class of rclcpp
class AddressBookPublisher : public rclcpp::Node
{

//constructor for the publisher node
public:
  AddressBookPublisher()

  //names the node
  : Node("address_book_publisher")
  {
    //initializes the publisher through the Template Function Call (which sets the msg type passed by this publisher)
        //of "create_publisher"and identifies the topic name and 
    address_book_publisher_ =
      this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);


    ////////////////////////////Lambda Function////////////////////////////////////
    // An anonymous function that we can treat like a variable rather than a function, useful for 
    // function pointers (essentially passing functions as parameters to other functions)

    // "auto" = void(*function_name)(parameter) : Defines that the variable is of type function
    // "[CAPTURES]" : Basically pass variables, objects etc that are used in the scope of th
      // the function but exist outside of the scope to allow for access
    // "(PARAMETERS)" : Parameters required by the lambda function
    // "void" : return type of the function
    auto publish_msg = [this]() -> void {
      
      //initializes the message of structure/type AddressBook
        auto message = more_interfaces::msg::AddressBook();

        //populate the data/attributes of the message
        message.first_name = "John";
        message.last_name = "Doe";
        message.phone_number = "1234567890";
        message.phone_type = message.PHONE_TYPE_MOBILE;

        //print contents of the msg
        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        //publish the messsage
        this->address_book_publisher_->publish(message);
      };


    //create a timer in the object that "calls" the publish_msg lambda function periodically at 1s
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

// apply smart (shared_ptr) pointers to objects/variables for memory maintenance
private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


//runtime things
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  rclcpp::shutdown();

  return 0;
}