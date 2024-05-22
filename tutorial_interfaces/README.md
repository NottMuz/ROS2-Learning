
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


### **3.) Write The Server Node**
- Will create the node ourselves by creating a new C++ file in the src directory of the package called:

  - **`add_two_ints_server.cpp`** 

- **Note**: To create a .cpp file, or any other for that matter, via the terminal open the file with the txt editor and name you want, edit it and then save it. The file will be made and then saved like that
- The required code for the file can be seen in the file in this repository


### **4.) Add The Excecutable For ROS To Find**
- This can be done by entering the **`CMakeLists.txt`** file in the package directory and adding the executable and naming it talker to allow the use of **`ros2 run`** for the node:

  - **`add_executable(server src/add_two_ints_server.cpp)`**
  - **`ament_target_dependencies(server rclcpp example_interfaces)`**
  
- To allow **`ros2 run`** to find the node add in the same file:
  **`install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})`**


## Steps to Making a C++ Client node

### **1.) Create The Package**
- Done in first part for the Publisher node

### **2.) Write The Client Node**
- Will create the node ourselves by creating a new C++ file in the src directory of the package called:

  - **`add_two_ints_client.cpp`** 

- **Note**: To create a .cpp file, or any other for that matter, via the terminal open the file with the txt editor and name you want, edit it and then save it. The file will be made and then saved like that
- The required code for the file can be seen in the file in this repository


### **3.) Adding Executable**
- This can be done by entering the **`CMakeLists.txt`** file in the package directory and adding the executable and naming it talker to allow the use of **`ros2 run`** for the node:

  - **`add_executable(client src/add_two_ints_client.cpp)`**
  - **`ament_target_dependencies(client rclcpp example_interfaces)`**
  
- To allow **`ros2 run`** to find the node add in the same file:

    - **`install(TARGETS
  client
  DESTINATION lib/${PROJECT_NAME})`**


  
  
  

  
