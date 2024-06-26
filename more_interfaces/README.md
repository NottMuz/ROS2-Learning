
# **Implementing Custom Message Definitions Package**

## What Is This For?

- This is a CMake ROS2 package that contains the essentials for understanding how to implement the custom msg/srv formats within the same package as our other nodes instead from a seperate package meant for messages

- Also remember that after creating the package you need to follow the standard procedure of building the package(s) using **`colcon`** and sourcing the local setup files

## Steps to Creating Our Custom Message Definitions

### **1.) Create The Package for both**
- Create with: **`ros2 pkg create --build-type ament_cmake more_interfaces`**
- Also create the directories for both message formats using **`mkdir msg`** in the main directory for the pkg

### **2.) Create a .msg file**
  - Need to create the format in the **`msg`** directory, and the file should be called **`AddressBook.msg`**. It's data structure is:
    - **`uint8 PHONE_TYPE_HOME=0`**
    - **`uint8 PHONE_TYPE_WORK=1`**
    - **`uint8 PHONE_TYPE_MOBILE=2`**

    - **`string first_name`**
    - **`string last_name`**
    - **`string phone_number`**
    - **`uint8 phone_type`**
    - **Note:** The uint8 constants are defined above and identify the phone types
 

### **3.) CMakeLists.txt Adjustment to Allow C++ and Python**
- To convert the interfaces you defined into language-specific code (like C++ and Python) so that they can be used in those languages, add the following lines:
  - **`find_package(rosidl_default_generators REQUIRED)`** #finds the packages that generates message code from msg/srv file
  - Declare the list of messages you want to generate (**Also, **`set`** can be used to neatly list all the interfaces**):
  - **set(msg_files`**
    - **`"msg/AddressBook.msg`**
  - Generate the messages:
  - **`rosidl_generate_interfaces(${PROJECT_NAME}`**
    - **`${msg_files})`**
  - **`ament_export_dependencies(rosidl_default_runtime)`**
  - **Note:** Implementing these allows us to generate source files from the **msg definition**

### **4.) package.xml Adjustment**
- Essentially the interfaces rely on the **`rosidl_default_generators`** package in order to generate language specific code (py or cpp) that our nodes can utilize, therefore we need to declare a **build tool dependency** on it
  - What is a build tool dependency you may be asking, it's akin to **`ament`** or **`colcon`**, where it essentially means that the package(s) rely on other tools/libraries during thier compilation, and a build tool will help contstruct those 'relationships' to the requirements. It does not run during the runtime of the application however.

- We also have a **runtime dependency** on the **`rosidl_default_runtime`** package, as it provides the necessary support to use the code from out ROS2 nodes
- For the syntax regarding how to add these dependencies in package.xml:
  - **`<buildtool_depend>rosidl_default_generators</buildtool_depend>`**
  - **`<exec_depend>rosidl_default_runtime</exec_depend>`**
  - **`<member_of_group>rosidl_interface_packages</member_of_group>`**
  - **Note:** The rosidl_interface_packages is the name of the dependency group that your package, tutorial_interfaces, should be associated with, declared using the <member_of_group> tag.

### **5.) Using an Interface From The Same Package**
- The code for the file can be seen in **`publish_address_book.cpp`** in the package source folder
 
### **6.) Build The Publisher**
- We need to create a new target for this node in the CMakeLists.txt:

  - **`find_package(rclcpp REQUIRED)`** #Locates package for dependencies

  - **`add_executable(publish_address_book src/publish_address_book.cpp)`** #adds an executable target by providing a name and its location
  - **`ament_target_dependencies(publish_address_book rclcpp)`** #automatically handles dependencies for the target

  - **`install(TARGETS`**
    - **` publish_address_book`**
     - **` DESTINATION lib/${PROJECT_NAME})`** # specifies which target to install and where
    
  ### **6.) Link against the interface**
-In order to use the messages generated in the same package we need to use the following CMake code:
  - **`rosidl_target_interfaces(publish_address_book
  ${PROJECT_NAME} "rosidl_typesupport_cpp")`**
- This finds the relevant generated C++ code from AddressBook.msg and allows your target to link against it.
- This step was not necessary when the interfaces being used were from a different package that was built independently. This CMake code is only **required when you want to use interfaces in the same package as the one in which they are defined**.

