
# **Parameters in Python ROS2**

## What Is This For?

- This is a Python ROS2 package that contains the essentials for understanding how to implement and adjust paramters for a node/package
- When making your own nodes you will sometimes need to add parameters that can be set from the launch file, as in they can be adjusted in the launch file itself for runtime.
- Also remember that after creating the package you need to follow the standard procedure of building the package(s) using **`colcon`** and sourcing the local setup files

## Steps to Creating Our Custom Message Definitions

### **1.) Create The Package for both**
- Create with: **`ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy`**

### **2.) package.xml Adjustment**
- We already resolved the required dependencies etc. in step 1 during the creation of the package
- Make sure to add the required documentation

### **3.) Writing the Python Node**
- The code for the file can be seen in **`python_parameters_node.py`** in the

### **4.) Adding It As An Executable in CMakeLists.txt**
- The code for the file can be seen in **`CMakeLists.txt`** in the main package directory, it is the standard code we have done before

### **5.) Adding a ParameterDescriptor()**
- Descriptors allow you to specify a text description of the parameter and its constraints, like making it read-only, specifying a range, etc. For that to work, the code in the constructor has to be changed to:
  - **`from rcl_interfaces.msg import ParameterDescriptor`**
  - **` my_parameter_descriptor = ParameterDescriptor(description='This parameter is mine!')`**
  - **`self.declare_parameter('my_parameter', 'world', my_parameter_descriptor)`**
- This provides the benefit of using **`ros2 param describe /minimal_param_node my_parameter`** to see the decription of the param. via the cmd line

### **5.) Adding an Entry Point**
-In the **`setup.py`** file add:
  - **` 'minimal_param_node = python_parameters.python_parameters_node:main',`**
- to the entry points console script

### **6.) Changing Via The Console**
- Make sure the node is running
- Open another terminal, source the setup files from inside ros2_ws again, and enter the following line:
  - **`ros2 param list`**
- There you will see the custom parameter my_parameter. To change it, simply run the following line in the console:
  - **`ros2 param set /minimal_param_node my_parameter earth`**

### **7.) Changing Via The Launch File**
-  Inside the ros2_ws/src/python_parameters/ directory, create a new directory called launch. In there, create a new file called cpp_parameters_launch.py
-  The code for it can be seen in the directory .py file
-   By adding the two lines below, we ensure our output is printed in our console.
  - **`output="screen",`**
  - **`emulate_tty=True,`**
- We will also need to make the adjustment to the setup.py file by adding to the **data_files** scope:
    - **`(os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))`**
