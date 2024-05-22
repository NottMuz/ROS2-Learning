
# **Simple C++ ROS2 Server & Client Package**

## What Is This For?

- This is a C++ ROS2 package that contains a service node and client node
- Check the ".cpp" files for both nodes to see the relavent 'notes'/documentation for the implementation of each in ROS2
- Also remember that after creating the packages you need to follow the standard procedure of building the package(s) using **`colcon`** and sourcing the local setup files

## Steps to Making a C++ Server Node

### **1.) Create The Package for both**
- Create with: **`ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces`**

- **`--dependencies`** will automatically add the necessary dependency lines to package.xml and CMakeLists.txt. **`example_interfaces`** is the package that includes the .srv file you will need to structure your requests and responses (.srv being the format of the reply/requests between nodes)
- It is "ament_cmake" for C++, would be "ament_python" for python
- Can also put "--node-name <node_name>" to generate a node as well

### **2.) Update package.xml**
- Due to using the **`--dependencies`** option in the previous step, the only thing that needs to be updated here is the documentation


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


  
  
  

  
