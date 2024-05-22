
# **Custom Message Definitions Package**

-**Note:** The packages that will use these msg/srv formats are the publisher and subscriber ones created previously, however minor adjustments need to be made to them to accomodate these new formats. That is not done in this repo.

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

### **4.) package.xml Adjustment **
- Essentially the interfaces rely on the **`rosidl_default_generators`** package in order to generate language specific code (py or cpp) that our nodes can utilize, therefore we need to declare a **build tool dependency** on it
  - What is a build tool dependency you may be asking, it's akin to **`ament`** or **`colcon`**, where it essentially means that the package(s) rely on other tools/libraries during thier compilation, and a build tool will help contstruct those 'relationships' to the requirements. It does not run during the runtime of the application however.

- We also have a **runtime dependency** on the **`rosidl_default_runtime`** package, as it provides the necessary support to use the code from out ROS2 nodes
- For the syntax regarding how to add these dependencies:
  - **`<depend>geometry_msgs</depend>`**
  - **`<buildtool_depend>rosidl_default_generators</buildtool_depend>`**
  - **`<exec_depend>rosidl_default_runtime</exec_depend>`**
  - **`<member_of_group>rosidl_interface_packages</member_of_group>`**
  - **Note:** The rosidl_interface_packages is the name of the dependency group that your package, tutorial_interfaces, should be associated with, declared using the <member_of_group> tag.
  
