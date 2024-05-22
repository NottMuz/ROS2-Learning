
# **Custom Message Definitions Package**

## What Is This For?

- This is a CMake ROS2 package that contains the essentials for understanding how to create your .msg or .srv interface definitions (ALTHOUGH CMAKE, CAN HOLD .py & .cpp)
- Check the .msg and .srv files to see the relavent 'notes'/documentation for the implementation of each in ROS2
- Also remember that after creating the package you need to follow the standard procedure of building the package(s) using **`colcon`** and sourcing the local setup files

## Steps to Creating Our Custom Message Definitions

### **1.) Create The Package for both**
- Create with: **`ros2 pkg create --build-type ament_cmake tutorial_interfaces`**
- Also create the directories for both message formats using **`mkdir srv msg`** in the main directory for the pkg

### **2.) Making Our Custom Definitions**
  #### 2.1) msg definition
  - Need to create the format in the **`msg`** directory, and the file should be called **`Num.msg`**. Only one line of code to declare its data structure is:
    - **`int64 num`**
  - We will create another msg format that uses a message format from another package, it will be called **`Sphere.msg`** with the following content:
    - **`geometry_msgs/Point center`**
    - **`float64 radius`**
  - Using another packages message means that when the data structure provided by the message is called upon, the data needed for the **`center`** is also required

  #### 2.2) srv definition
 - Need to create the format in the **`srv`** directory, and the file should be called **`AddThreeInts.srv`**. Its data structure is:
    - **`int64 a`**
    - **`int64 b`**
    - **`int64 c`**
    - **`---`**
    - **`int64 sum`**
    - **Note:** The line with **`---`** seperates the format of request from that of the reply

### **3.) CMakeLists.txt Adjustment to Allow C++ and Python**
- To convert the interfaces you defined into language-specific code (like C++ and Python) so that they can be used in those languages, add the following lines:
  - **`find_package(geometry_msgs REQUIRED)`**
  - **`find_package(rosidl_default_generators REQUIRED)`**
  - **`rosidl_generate_interfaces(${PROJECT_NAME}`**
    - **`"msg/Num.msg"`**
    - **`"msg/Sphere.msg"`**
    - **`"srv/AddThreeInts.srv"`**
    - **`DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg )`**



### **4.) Add The Excecutable For ROS To Find**
- This can be done by entering the **`CMakeLists.txt`** file in the package directory and adding the executable and naming it talker to allow the use of **`ros2 run`** for the node:

  - **`add_executable(server src/add_two_ints_server.cpp)`**
  - **`ament_target_dependencies(server rclcpp example_interfaces)`**
  
- To allow **`ros2 run`** to find the node add in the same file:
  **`install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})`**



  

  
