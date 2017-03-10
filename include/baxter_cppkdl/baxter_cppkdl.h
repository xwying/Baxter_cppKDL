#include <ros/ros.h>
#include <std_msgs/String.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#ifndef CPPKDL_H
#define CPPKDL_H

using namespace std;
using namespace KDL;

namespace CPPKDL {
class baxter_kinematics
{
  public:
    baxter_kinematics(std::string limb);
    void forward_kinematics(double (&result)[7]);
    void forward_kinematics(double (&result)[7], double joint_values[7]);
    void inverse_kinematics(double (&result)[7], double position[3]);
    void inverse_kinematics(double (&result)[7], double position[3], double orientation[4]);
    void inverse_kinematics(double (&result)[7], double position[3], double orientation[4], double seed[7]);
  private:
    double q_l[7];
    double q_r[7];
    double q[7];
    bool joint_state_ready;
    int num_jnts;
    std::string limb;
    KDL::Tree kdl_tree;
    KDL::Chain arm_chain;
    void forward_kinematics_function(double (&result)[7], double joint_values[7]);
    void inverse_kinematics_function(double (&result)[7], double position[3], double orientation[4], double seed[7]);
    void Joint_state_callback(sensor_msgs::JointState mMsg); 
};

}

#endif
