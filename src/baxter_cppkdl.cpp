#include <baxter_cppkdl.h>

namespace CPPKDL {

using namespace std;
using namespace KDL;

baxter_kinematics::baxter_kinematics(std::string _limb)
{
  limb = _limb;
  joint_state_ready = 0;

  ros::NodeHandle node;
  //Create a subscriber for joint state
  ros::Subscriber Joint_States_sub = node.subscribe("/robot/joint_states", 100, &baxter_kinematics::Joint_state_callback, this);
  //Read the URDF from param sever  
  if (!kdl_parser::treeFromParam("robot_description", kdl_tree)){
    ROS_ERROR("Failed to construct kdl tree");
  }  
  //Get chain from kdl tree
  std::string base_link = "base";
  std::string tip_link = limb + "_gripper";  
  kdl_tree.getChain(base_link, tip_link, arm_chain);
  //Get number of Joints
  num_jnts = arm_chain.getNrOfJoints();
  
  //Wait for first message from joint_state subscriber arrives
  while(ros::ok() && !joint_state_ready){
    ros::spinOnce();
  }
}

void baxter_kinematics::Joint_state_callback(sensor_msgs::JointState mMsg){
  int i;
  //setting joints in the right order to feed the jacobian
	//JointState order -> e0, e1, s0, s1, w0, w1, w2
	//desired order -> s0, s1, e0, e1, w0, w1, w2
  int t=0;
  if (limb=="right"){
    t = 7;
  }
	for(i = 0; i < 2 ; i++){
		// s0 and s1
    q[i] = (double)(mMsg.position[i + 4 + t]);
	}

	for(i = 2; i < 4 ; i++){ 
		// e0 and e1
    q[i] = (double)(mMsg.position[i + t]);
	}

	for(i = 4; i < 7 ; i++){ 
		// w0, w1, w2
    q[i] = (double)(mMsg.position[i + 2 + t]);
	}
  joint_state_ready=1;
}



void baxter_kinematics::forward_kinematics(double (&result)[7])
{
  forward_kinematics_function(result, NULL);
}

void baxter_kinematics::forward_kinematics(double (&result)[7], double joint_values[7])
{
  forward_kinematics_function(result,joint_values);
}

void baxter_kinematics::inverse_kinematics(double (&result)[7], double position[3])
{
  inverse_kinematics_function(result, position, NULL, NULL);
}

void baxter_kinematics::inverse_kinematics(double (&result)[7], double position[3], double orientation[4])
{
  inverse_kinematics_function(result, position, orientation, NULL);
}

void baxter_kinematics::inverse_kinematics(double (&result)[7], double position[3], double orientation[4], double seed[7])
{
  inverse_kinematics_function(result, position, orientation, seed);
}

void baxter_kinematics::forward_kinematics_function(double (&result)[7], double joint_values[7]=NULL)
{
  //Create KDL Solver
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(arm_chain);
  //Create Joint Array for calculation
  KDL::JntArray	joint_positions = JntArray(num_jnts);
  int i;

  if (joint_values == NULL){
    //Read the joint values from subscriber
    for(i = 0; i<7; i++)
    {
      joint_positions(i)=q[i];
      //printf("%g\n",q[i]);
    }
  }
  else{
    for(i = 0; i<7; i++)
    {
      joint_positions(i)=joint_values[i];
    }
  }
  //Create the frame that will contain the results
  KDL::Frame end_frame;
  bool fk_status;
  fk_status = fksolver.JntToCart(joint_positions,end_frame); 
  KDL::Vector pos;
  KDL::Rotation rotation = Rotation(end_frame.M);
  pos = end_frame.p;
  double rot[3];
  rotation.GetQuaternion(rot[0],rot[1],rot[2],rot[3]);

  //Modifying the output array 
  for(i=0; i<3; i++)
  {
    result[i]=pos(i);
  }
  for(i=0; i<4; i++)
  {
    result[i+3]=rot[i];
  }
}

void baxter_kinematics::inverse_kinematics_function(double (&result)[7], double position[3], double orientation[4] = NULL, double seed[7]=NULL)
{
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(arm_chain);
  ChainIkSolverVel_pinv iksolver_v = 	ChainIkSolverVel_pinv(arm_chain);
  ChainIkSolverPos_NR iksolver_p = ChainIkSolverPos_NR(arm_chain,fksolver,iksolver_v);
  KDL::Vector pos = Vector(position[0],position[1],position[2]);
  KDL::Rotation rot = Rotation();
  if (orientation != NULL){
    rot = rot.Quaternion(orientation[0],orientation[1],orientation[2],orientation[3]);
  }
  //Populate seed wit current angles if not provided
  KDL::JntArray seed_array = JntArray(num_jnts);
  int i;  
  if (seed != NULL){
    for(i=0; i<num_jnts; i++)
    {
      seed_array(i)=seed[i];
    }
  }
  else{
    for(i=0; i<num_jnts; i++)
    {
      seed_array(i)=q[i];
    }
  }
  //Make IK Call
  KDL::Frame goal_pose;
  if (orientation != NULL)
  {
    goal_pose = Frame(rot,pos);
  }
  else{
    goal_pose = Frame(pos);
  }
  KDL::JntArray result_angles = JntArray(num_jnts);
  bool ik_status;
  ik_status = iksolver_p.CartToJnt(seed_array, goal_pose, result_angles);  

  for(i=0; i<7; i++)
  {
    result[i]=result_angles(i);
  }
}
}

