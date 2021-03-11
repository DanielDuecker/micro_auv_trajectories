#ifndef RVIZ_HPP
#define RVIZ_HPP
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include "CommonMath/Vec3.hpp"
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
using namespace CommonMath;
class rviz
{
public:
    rviz(ros::NodeHandle* nodehandle);
    void publishTrajectory(const RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator &ptr, const double tmr,const int coloroption);
    void publishGoal(const Vec3 pose);
    void publishBoatPosition( const CommonMath::Vec3 position, const int counter);
    void publishSingleObstacle( const CommonMath::Vec3 position, const double radius);
    void publishSingleObstacle2( const CommonMath::Vec3 position, const double radius);
    void publishSingleObstacle3( const CommonMath::Vec3 position, const double radius);
    void publishSingleObstacle4( const CommonMath::Vec3 position, const double radius);
    void publishRectangle( const CommonMath::Vec3 position, const double radius);
    void publishWalls();
    void publishVelocityText(const CommonMath::Vec3 velocity);
    void publishDataText(const CommonMath::Vec3 data);
private:
    ros::NodeHandle nh;
    ros::Publisher rviz_traject;
    ros::Publisher goal_visual;
    ros::Publisher boat_position;
    ros::Publisher single_obstacle;
    ros::Publisher single_obstacle2;
    ros::Publisher single_obstacle3;
        ros::Publisher single_obstacle4;
    ros::Publisher tank_walls;
    ros::Publisher velo_info;
    ros::Publisher data_info;
    ros::Publisher rectangle;
    
    
    void initializePublisher();
    
};

#endif // RVIZ_HPP
