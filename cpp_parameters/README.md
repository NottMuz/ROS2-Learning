
# **Parameters in C++ ROS2**

## What Is This For?

- This is a CMake ROS2 package that contains the essentials for understanding how to implement and adjust paramters for a node/package
- When making your own nodes you will sometimes need to add parameters that can be set from the launch file.
- Also remember that after creating the package you need to follow the standard procedure of building the package(s) using **`colcon`** and sourcing the local setup files

## Steps to Creating Our Custom Message Definitions

### **1.) Create The Package for both**
- Create with: **`ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp`**

### **2.) package.xml Adjustment**
- We already resolved the required dependencies etc. in step 1 during the creation of the package
- Make sure to add the required documentation

### **3.) Writing the C++ Node**
- The code for the file can be seen in **`cpp_parameters_node.cpp`** in the

### ** .) Adding It As An Executable in CMakeLists.txt**
- The code for the file can be seen in **`CMakeLists.txt`** in the main package directory, it is the standard code we have done before
 

