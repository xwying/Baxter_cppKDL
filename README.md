# Baxter cppKDL
A Kinematic Solver for Baxter Robot in C++.

# Introduction
Baxter cppKDL is a C++ library that offers forward kinematics and inverse kinematic calculation. We got inspired by [Baxter_pyKDL](http://sdk.rethinkrobotics.com/wiki/Baxter_PyKDL) and tried to build a similiar library for C++ users. It aimed to reduce the difficuty of using orocos KDL for the kinematic and dynamic analysis of Baxter in C++.

# Prerequisite
This library uses functions in orocos KDL(Kinematics and Dynamics Library). Thus, this package has to be installed. Please refer to [http://www.orocos.org/kdl/installation-manual] for more information.

# Build the baxter_cppkdl package
```
 $ cd ~/ros_ws/src
 $ git clone https://github.com/xwying/Baxter_cppKDL.git
 $ cd ~/ros_ws
 $ source ./baxter.sh
 $ catkin_make
```

# Class
+ baxter_kinematics(limb)
```
Arguments:limb(required), it could be 'left' or 'right'.
Description: Initialize the class with specific limb.
```
# Method
+ forward_kinematics(result, joint_value(optional))
```
Arguments: result(array)(required), joint_value(array)(optional)
Description: Solves the forward kinematics, if the joint value is unspecified, it will read the current joint angle from Baxter. The result will be written to result_array.
``` 
+ inverse_kinematics(result, position, orientation(optional), seed(optional))
```
Arguments:
    result(array)(required)
    position(array)(required) - Cartesian Position, Orientation (optional) [x, y, z]
    orientation(array)(optional) - Quaternion Orientation [i, j, k, w]
    seed(array)(optional) - Seeded joint angles for solver. Searchs out from seeded joint angles for IK solution [s0, s1, e0, e1, w0, w1, w2] 
    Description: Solves the inverse kinematics using, providing joint angles given a Cartesian pose. The result will be written to result_array.
```

# Example
We also wrote an Example about how to use this libray, please refer to [Baxter_cppKDL_Example](https://github.com/xwying/Baxter_cppKDL_Example) repository.

# Author
+ Xiaowen Ying [@xwying](https://github.com/xwying)
+ Yubo Fan [@ybfan94](https://github.com/ybfan94)

This is our first ROS project. If you would like to give us some comments, we can be reached by sslf[at]foxmail[dot]com or ybfan910[at]163[dot]com.
