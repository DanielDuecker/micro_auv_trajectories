#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/Imu.h"
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/HippocampusControl.h>
#include "hippocampus_test/data_subscriber.hpp"
#include "hippocampus_test/rviz.hpp"
#include <chrono>
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"
#include "hippocampus_test/TrajectoryCreator.hpp"
#include <math.h> 
#include <time.h>
#include "hippocampus_test/BallCatch.hpp" 
#include "hippocampus_test/SingleTracking.hpp" 
#include "hippocampus_test/ObstacleAvoidance.hpp" 
using namespace std;
using namespace RapidQuadrocopterTrajectoryGenerator;

//Two simple helper function to make testing easier
const char* GetInputFeasibilityResultName(RapidTrajectoryGenerator::InputFeasibilityResult fr)
{
  switch(fr)
  {
  case RapidTrajectoryGenerator::InputFeasible:             return "Feasible";
  case RapidTrajectoryGenerator::InputIndeterminable:       return "Indeterminable";
  case RapidTrajectoryGenerator::InputInfeasibleThrustHigh: return "InfeasibleThrustHigh";
  case RapidTrajectoryGenerator::InputInfeasibleThrustLow:  return "InfeasibleThrustLow";
  case RapidTrajectoryGenerator::InputInfeasibleRates:      return "InfeasibleRates";
  }
  return "Unknown!";
};

const char* GetStateFeasibilityResultName(RapidTrajectoryGenerator::StateFeasibilityResult fr)
{
  switch(fr)
  {
  case RapidTrajectoryGenerator::StateFeasible:   return "Feasible";
  case RapidTrajectoryGenerator::StateInfeasible: return "Infeasible";
  }
  return "Unknown!";
};

double calculateThrust(double force){
    double thrust = 0.0;
    double  a = 2.26892684e-05,
            b = -8.14731295e-04,
            c = 1.58229761e-01 - force;
    double d = b * b  - 4 * a * c;
    double x1 = 0.0;
    double x2 = 0.0;
   
    if (d < 0){
        double dummy=0.0;
            
    }else if (d == 0){
            x1 = -b / (2 * a);
        
    }else if (d > 0){ 
        
            x1 = (-b + sqrt(d)) / (2 * a);
            x2 = (-b - sqrt(d)) / (2 * a);
    }
    if (x1 >0 and x1>x2){
        thrust= x1/500;
    }
    if (x2 >0 and x2>x1){
        thrust = x2/500;
    }

    return thrust;
};


//---------------------------------------------------------------------------------

int main(int argc, char **argv)
{

  ros::init(argc, argv, "control");
  ros::NodeHandle nh;
  BallCatch catch_the_ball(&nh);
  //SingleTracking single(&nh);
 // ObstacleAvoidance avoidance(&nh);
  ros::Rate loop_rate(30);
  //catch_the_ball.ControlFunction();
 
  
//------------------------------

/*  
  data_subscriber boatdata(&nh);
  rviz rviz_class(&nh);
  TrajectoryCreator trajcreator;
  
  ros::Publisher publish_control = nh.advertise<mavros_msgs::HippocampusControl>("hippocampus/desired_values", 1);
  ros::Rate loop_rate(30);
    
  int count = 0;
  
  //Define the trajectory starting state:
  Vec3 pos0 = Vec3(0, 0, 0); //position
  Vec3 vel0 = Vec3(0, 0, 0); //velocity
  Vec3 acc0 = Vec3(0, 0, 0); //acceleration

  //define the goal state:
  Vec3 posf = Vec3(1, 1, 0); //position
  Vec3 velf = Vec3(0, 0, 0); //velocity
  Vec3 accf = Vec3(0, 0, 0); //acceleration

  //define the duration:
  double Tf = 5.0;

  double fmin = 0;//[m/s**2]
  double fmax =10;//[m/s**2]
  double wmax = 20;//[rad/s]
  double minTimeSec = 0.02;//[s]
  //Define how gravity lies in our coordinate system
  Vec3 gravity = Vec3(0,0,-9.81);//[m/s**2]
  Vec3 floorPos = Vec3(0,0,0);//any point on the boundary
  Vec3 floorNormal = Vec3(0,0,1);//we want to be in this direction of the boundary
  mavros_msgs::HippocampusControl msg;
  float desiredThrust = 0.0;
  Vec3 desiredAxis=Vec3 (0,1,0);
  ros::WallTime start_, go_,start2_;

  start_ = ros::WallTime::now() ;
  start2_= ros::WallTime::now();
  
  
  
  while (ros::ok())
  {
    go_=ros::WallTime::now();
    
    double temp = ( start_.toSec() - (start2_.toSec()) + (go_.toSec()) );
    double execution_time =( (go_.toSec()) - (start_.toSec()) );
    ROS_INFO_STREAM("Exectution time (s): " << (execution_time ) );
//Update Data
   // pos0 = boatdata.GetPosition();
    //vel0 = boatdata.GetVelocity();
    //acc0 = boatdata.GetAcceleration() * 0;
    std::vector<double> quat = boatdata.GetOrientation();
    
    
//Generate Trajectory
    trajcreator.SetGoalPosition(posf);
    trajcreator.SetGoalVelocity(velf);
    trajcreator.SetGoalAcceleration(accf);
    RapidTrajectoryGenerator traj = trajcreator.GenerateTrajectories(pos0,vel0,acc0,3,Tf);
    trajcreator.DeleteTrajectoryList();
  
    rviz_class.publishTrajectory(traj);

//------------------------
    desiredAxis = traj.GetNormalVector((2.0));
    desiredThrust = calculateThrust(traj.GetThrust(2.0));
   
    //cout << "run\n" << desiredThrust;
    msg.frame_stamp = ros::Time::now();
    msg.thrust = desiredThrust;
    msg.roll_effort = desiredAxis[0];
    msg.pitch_effort= desiredAxis[1];
    msg.yaw_effort  = desiredAxis[2];
    publish_control.publish(msg);
    ROS_INFO("Accel %f %f %f:", desiredAxis[0],desiredAxis[1],desiredAxis[2]);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

 */ 
  return 0;
}

