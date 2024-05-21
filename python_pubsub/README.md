
# **Simple Python ROS2 Publisher & Subscriber Package**

## What Is This For?

- This is a Python ROS2 package that contains a subscriber node and publisher node
- Check the ".py" files for both nodes to see the relavent 'notes'/documentation for the implementation of each in ROS2
- Also remember that after creating the packages you need to follow the standard procedure of building the package(s) using **`colcon`** and sourcing the local setup files

## Steps to Making a C++ Publisher Node

### **1.) Create The Package for both**
- Create with: **` ros2 pkg create --build-type ament_python <pkg_name>`**
- It is "ament_python" for python, would be "ament_cmake" for C++ 
- Can also put "--node-name <node_name>" to generate a node as well

### **2.) Write The Publisher Node**
- Can create ourselves, in this case we will source one to our 'src' folder with:
  
**`wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py`**

- Provides us with the Python file for our publisher


#### **2.5.) Examination of The Python file**
- Can be seen within:

**`ROS2-Learning/python_pubsub/python_pubsub/publisher_member_function.py`**


### **3.) Adding Dependencies (package.xml)**
- Due to the use of the 'rosclient' and the 'std_msgs' namespaces (imported in the .py file), we need to resolve those dependencies so they can be called/found correctly during runtime
- This can be done by entering the **`package.xml`** file in the package directory and adding:

**`<exec_depend>rclpy</exec_depend>`**

**`<exec_depend>std_msgs</exec_depend>`**

, also make sure to fill in the documentation for the <description>, <maintainer> and <license> tags
  
### **4.) Adding an Entry Point**
- "Adding entry points in a Python package:

    Defines executable scripts.
    Simplifies script execution.
    Ensures proper package distribution.
    Integrates your package with ROS 2 tools and workflows.

This setup allows your ROS 2 nodes to be easily executed and managed within the ROS 2 ecosystem."

- This can be done by entering the **`setup.py`** file in the package directory and adding:
  
**`entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},`**

, also make sure to fill in the documentation

### **5.) Checking the setup.cfg File**
- Make sure the config file is correctly populated before building

## Steps to Making a Python Subscriber node

### **1.) Create The Package**
- Done in first part for the Publisher node

### **2.) Write The Subscriber Node**
- Can create ourselves, in this case we will source one to our 'ros2_ws/src/py_pubsub/py_pubsub' folder with:

  **`wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py`**

- Provides us with the Python file for our subscriber


#### **2.5.) Examination of The Python file**
- Can be seen within: **`ROS2-Learning/python_pubsub/python_pubsub/subscriber_member_function.cpp`**


### **3.) Adding Dependencies (package.xml)**
-  Resolved in the publisher portion above

### **4.) Adding an Entry Point**
- "Adding entry points in a Python package:

    Defines executable scripts.
    Simplifies script execution.
    Ensures proper package distribution.
    Integrates your package with ROS 2 tools and workflows.

This setup allows your ROS 2 nodes to be easily executed and managed within the ROS 2 ecosystem."

- This can be done by entering the **`setup.py`** file in the package directory and adding:
  
**`entry_points={
        'console_scripts': [
        'listener = py_pubsub.subscriber_member_function:main',
        'talker = py_pubsub.publisher_member_function:main',
        ],
},`**
  
  
  

  
