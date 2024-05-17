
# **Simple C++ ROS2 Publisher & Subscriber Package**

## What Is This For?

- This is a C++ ROS2 package that contains a subscriber node and publisher node
- Check the ".cpp" files for both nodes to see the relavent 'notes'/documentation for the implementation of each in ROS2

## Steps to Making a C++ Publisher Package

### **1.) Create The Package**
- Create with: **` ros2 pkg create --build-type ament_cmake <pkg_name>`**
- It is "ament_cmake" for C++, would be "ament_python" for python
- Can also put "--node-name <node_name>" to generate a node as well

### **2.) Write The Publisher Node **
- Can create ourselves, in this case we will source one with: **` wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_publisher/member_function.cpp`**
- Provides us with the C++ file for our publisher


### **2.5.) Examination of The C++ file**
- Can be seen within: **`ROS2-Learning/cpp_pubsub/src/publisher_member_function.cpp`**


### **3.) Adding Dependencies (package.xml) **
- Due to the use of the 'rosclient' and the 'std_msgs' namespaces (imported in the .cpp file), we need to resolve those dependencies so they can be called/found correctly during runtime
- This can be done by entering the **`package.xml`** file in the package directory and adding:
**`<depend>rclcpp</depend>`**
**`<depend>std_msgs</depend>`**
  after **`ament_cmake`**, also make sure to fill in the documentation for the <description>, <maintainer> and <license> tags
  
### **4.) Adding Dependencies (CMakeLists.txt) **
- Due to the use of the 'rosclient' and the 'std_msgs' namespaces (imported in the .cpp file), we need to resolve those dependencies so they can be called/found correctly during runtime
- This can be done by entering the **`package.xml`** file in the package directory and adding:
**`<depend>rclcpp</depend>`**
**`<depend>std_msgs</depend>`**
  after **`ament_cmake`**, also make sure to fill in the documentation for the <description>, <maintainer> and <license> tags

  
