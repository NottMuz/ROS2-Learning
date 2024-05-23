# ROS2-Learning

Just a repository where I am learning about ROS2

# General C++ Notes
- **`shared_ptr`** : Attaching a shared pointer to an object/variable allows us to bypass have to manually delete/create the variables in memory, called a "smart pointer". Basically keeps track of how many references there are to the var. and once reaches zero (example we leave a scope) it deletes the memory adress of the actual variable itself.
- **`<...>`** : Template function call allowing for any parameter/data type to be used in the function
  

## 1.1) cpp_pubsub
- C++ package for learning about the Publisher-Subscriber communication protocol within ROS2

## 1.2) cpp_srvcli
- C++ package for learning about the Service-Client communication protocol within ROS2

## 2.1) python_pubsub
- Python package for learning about the Publisher-Subscriber communication protocol within ROS2

## 2.2) python_srvcli
- Python package for learning about the Service-Client communication protocol within ROS2

## 3.1) tutorial_interfaces
- CMake package that for learning about custom message (.msg and .srv) formats

## 3.2) more_interfaces
- CMake package that for learning about implementing custom interfaces
