
# **Simple C++ ROS2 Service & Client Package**

## What Is This For?

- This is a C++ ROS2 package that contains a service node and client node
- Check the ".cpp" files for both nodes to see the relavent 'notes'/documentation for the implementation of each in ROS2
- Also remember that after creating the packages you need to follow the standard procedure of building the package(s) using **`colcon`** and sourcing the local setup files

## Steps to Making a C++ Service Node

### **1.) Create The Package for both**
- Create with: **`ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces`**

- **`--dependencies`** will automatically add the necessary dependency lines to package.xml and CMakeLists.txt. **`example_interfaces`** is the package that includes the .srv file you will need to structure your requests and responses (.srv being the format of the reply/requests between nodes)

- It is "ament_cmake" for C++, would be "ament_python" for python
- Can also put "--node-name <node_name>" to generate a node as well

### **2.) Update package.xml**
- Due to using the **`--dependencies`** option in the previous step, the only thing that needs to be updated here is the documentation


### **2.) Write The Service Node**
- Will create the node ourselves by creating a new C++ file in the src directory of the package called:

**`add_two_ints_server.cpp`** 

-**Note**: To create a .cpp file, or any other for that matter, via the terminal open the file with the txt editor and name you want, edit it and then save it. The file will be made and then saved like that
- The required code for the file can be seen in the file in this repository


#### **2.5.) Examination of The C++ file**
- Can be seen within: **`ROS2-Learning/cpp_pubsub/src/publisher_member_function.cpp`**


### **3.) Adding Dependencies (package.xml)**
- Due to the use of the 'rosclient' and the 'std_msgs' namespaces (imported in the .cpp file), we need to resolve those dependencies so they can be called/found correctly during runtime
- This can be done by entering the **`package.xml`** file in the package directory and adding:
**`<depend>rclcpp</depend>`**
**`<depend>std_msgs</depend>`**
  after **`ament_cmake`**, also make sure to fill in the documentation for the <description>, <maintainer> and <license> tags
  
### **4.) Adding Dependencies (CMakeLists.txt)**
- Due to the same reasons as before we need to make sure to do it here
- This can be done by entering the **`CMakeLists.txt`** file in the package directory and adding:
**`find_package(rclcpp REQUIRED)`**
**`find_package(std_msgs REQUIRED)`**
  after **`find_package(ament_cmake REQUIRED)`**, also make sure to fill in the documentation

### **5.) Making Sure ROS2 Can Find The Executable/Node**
- This can be done by entering the **`CMakeLists.txt`** file in the package directory and adding the executable and naming it talker to allow the use of **`ros2 run`** for the node:
**`add_executable(talker src/publisher_member_function.cpp)`**
**`find_package(std_msgs REQUIRED)`**
- To allow **`ros2 run`** to find the node add in the same file:
  **`install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME}) `**

## Steps to Making a C++ Subscriber node

### **1.) Create The Package**
- Done in first part for the Publisher node

### **2.) Write The Subscriber Node**
- Can create ourselves, in this case we will source one to our 'src' folder with: **`wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_subscriber/member_function.cpp`**
- Provides us with the C++ file for our subscriber


#### **2.5.) Examination of The C++ file**
- Can be seen within: **`ROS2-Learning/cpp_pubsub/src/subscriber_member_function.cpp`**


### **3.) Adding Dependencies (package.xml & CMakeList.txt)**
-  Resolved in the publisher portion above

### **4.) Making Sure ROS2 Can Find The Executable/Node**
- This can be done by entering the **`CMakeLists.txt`** file in the package directory and adding the executable and naming it talker to allow the use of **`ros2 run`** for the node:
**`add_executable(listener src/subscriber_member_function.cpp)`**
**`ament_target_dependencies(listener rclcpp std_msgs)`**
- To allow **`ros2 run`** to find the node add in the same file:
  **`install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})`**
  
  
  

  
