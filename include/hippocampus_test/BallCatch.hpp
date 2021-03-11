#ifndef BALLCATCH_HPP
#define BALLCATCH_HPP
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/Imu.h"
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/HippocampusDesired.h>
#include <mavros_msgs/HippocampusOutput.h>
#include "hippocampus_test/data_subscriber.hpp"
#include "hippocampus_test/rviz.hpp"
#include <chrono>
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"
#include "hippocampus_test/TrajectoryCreator.hpp"
#include <math.h> 
#include <time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <random>
#include <fstream>
#include <memory>
#include <vector>
#include "CommonMath/ConvexObj.hpp"
#include "CommonMath/Sphere.hpp"
#include "CommonMath/Vec3.hpp"
#include "RapidCollisionDetection/CollisionChecker.hpp"
using namespace RapidQuadrocopterTrajectoryGenerator;

using namespace std;
using namespace std::chrono;
using namespace CommonMath;
using namespace RapidCollisionChecker;


class BallCatch
{
public:
    BallCatch(ros::NodeHandle* nodehandle);
    
   
    void CatchTheBall();
    void ControlFunction();
private:
    ros::NodeHandle nh;
    ros::Publisher publish_desired_values;
    ros::Publisher output_values;
    ros::WallTime timer;
    RapidTrajectoryGenerator traj;
    data_subscriber boatdata; 
    rviz rviz_publisher;
    TrajectoryCreator trajcreator;
    void SetTimer();
    void Reset();
    void initializePublisher();
    void PublishDesiredValues(const double thrust_value, const CommonMath::Vec3 axis);
    double calculateThrust(const double force);
    void GoToStart();
    void OrientateToGoal();
    void calculateGoal();
    void closeGoal();
    void writeData(const CommonMath::Vec3 des_pos,const CommonMath::Vec3 curr_pos,const CommonMath::Vec3 des_vel,
                   const CommonMath::Vec3 curr_vel,double des_time,double final_time);
    //void ControlFunction();
    int counter;
    
    bool IsAtStart;
    bool PerformCatching;
    bool SetTheTime;
    bool OrientateTowardsGoal;
    bool CloseToGoal;
    int whichGoal;
    
    double DesiredThrust;
    double execution_time;
    double trajectory_duration, Tf;
    CommonMath::Vec3 DesiredAxis,StartPosition,posf;
    CommonMath::Vec3 goalPosition, goalVelocity, goalAcceleration;
    
};

#endif // BALLCATCH_HPP
