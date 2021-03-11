#ifndef DATA_SUBSCRIBER_HPP
#define DATA_SUBSCRIBER_HPP
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include "CommonMath/Vec3.hpp"
#include "quaternion/quaternion.h"
#include <math.h>
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/HippocampusControl.h>
#include <mavros_msgs/HippocampusCurrentaxis.h>
//#include <mavros/mavros_plugin.h>
//#include <mavros_msgs/HippocampusControl.h>
using namespace std;
using namespace quaternion;
using namespace CommonMath;


class data_subscriber
{
public:
    data_subscriber(ros::NodeHandle* nodehandle);
    
    CommonMath::Vec3 GetPosition(void)     const {return position;};
    CommonMath::Vec3 GetVelocity(void)     const {return velocity;};
    CommonMath::Vec3 GetAcceleration(void)     const {return acceleration;};
    CommonMath::Vec3 GetAngularVelocity(void)     const {return angular_velocity;};
    CommonMath::Vec3 GetCurrentAxis(void)     const {return current_axis;};
    std::vector<double> GetOrientation(void) const {return orientation;};
    ros::Time getTimeStamp(void) const{return timeStamp;};
private:
  ros::NodeHandle nh;
  ros::Subscriber position_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber acceleration_subscriber_;
  ros::Subscriber angular_velocity_subscriber_;
  ros::Subscriber current_axis_subscriber_;

  CommonMath::Vec3 position;
  CommonMath::Vec3 velocity;
  CommonMath::Vec3 acceleration;
  CommonMath::Vec3 angular_velocity;
  CommonMath::Vec3 current_axis;
  std::vector<double> orientation{0,0,0,0};
  
  void initializeSubscribers();
  void PositionCallback(const geometry_msgs::PoseStamped::ConstPtr &pos);
  void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &velo);
  void AccelerationCallback(const sensor_msgs::Imu::ConstPtr &accel);
  void AngularVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &angu);
  void CurrentAxisCallback(const mavros_msgs::HippocampusCurrentaxis::ConstPtr &axis);
  ros::Time timeStamp;
};


#endif // DATA_SUBSCRIBER_HPP
