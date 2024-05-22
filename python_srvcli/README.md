
# **Simple Python ROS2 Server & Client Package**

## What Is This For?

- This is a Python ROS2 package that contains a service node and client node
- Check the ".py" files for both nodes to see the relavent 'notes'/documentation for the implementation of each in ROS2
- Also remember that after creating the packages you need to follow the standard procedure of building the package(s) using **`colcon`** and sourcing the local setup files

## Steps to Making a Python Server Node

### **1.) Create The Package for both**
- Create with: **`ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces`**

- **`--dependencies`** will automatically add the necessary dependency lines to package.xml and CMakeLists.txt. **`example_interfaces`** is the package that includes the .srv file you will need to structure your requests and responses (.srv being the format of the reply/requests between nodes)
- It is "ament_cmake" for C++, would be "ament_python" for python
- Can also put "--node-name <node_name>" to generate a node as well

### **2.) Update package.xml**
- Due to using the **`--dependencies`** option in the previous step, the only thing that needs to be updated here is the documentation

### **3.) Write The Server Node**
- Will create the node ourselves by creating a new .py file in the package_name directory of the package called:

  - **`service_member_function.py`** 

- **Note**: To create a .py file, or any other for that matter, via the terminal open the file with the txt editor and name you want, edit it and then save it. The file will be made and then saved like that
- The required code for the file can be seen in the file in this repository


### **4.) Adding an Entry Point**
- This can be done by entering the **`setup.py`** file in the package directory and adding the executable and naming it service to allow the use of **`ros2 run`** for the node:

  - **`'service = py_srvcli.service_member_function:main',`**

- Done between the **`console_scripts:`** brackets


## Steps to Making a Python Client node

### **1.) Create The Package**
- Done in first part for the Publisher node

### **2.) Write The Client Node**
- Will create the node ourselves by creating a new .py file in the package_name directory of the package called:

  - **`client_member_function.py`** 

- **Note**: To create a .py file, or any other for that matter, via the terminal open the file with the txt editor and name you want, edit it and then save it. The file will be made and then saved like that
- The required code for the file can be seen in the file in this repository

### **3.) Adding an Entry Point**
- This can be done by entering the **`setup.py`** file in the package directory and adding the executable and naming it service to allow the use of **`ros2 run`** for the node:

  - **`'client = py_srvcli.client_member_function:main'`**

- Done between the **`console_scripts:`** brackets


  
  
  

  
