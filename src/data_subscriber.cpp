
#include "hippocampus_test/data_subscriber.hpp"

data_subscriber::data_subscriber(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    ROS_INFO("in class constructor of DataSuscriber");
    initializeSubscribers(); 
}  
    void data_subscriber::initializeSubscribers(){
        
        ROS_INFO("Initializing Subscribers");
        //position_subscriber_ = nh.subscribe("/uuv00/mavros/local_position/pose_NED2", 1, &data_subscriber::PositionCallback,this);
        position_subscriber_ = nh.subscribe("/uuv00/pose_px4", 1, &data_subscriber::PositionCallback,this);  
        velocity_subscriber_ = nh.subscribe("uuv00/mavros/local_position/velocity_localNED2", 1, &data_subscriber::VelocityCallback,this); 
        acceleration_subscriber_ = nh.subscribe("/uuv00/mavros/imu/data_NED2", 1, &data_subscriber::AccelerationCallback,this); 
        angular_velocity_subscriber_ = nh.subscribe("/uuv00/mavros/local_position/velocity_bodyNED2", 1, &data_subscriber::AngularVelocityCallback,this); 
        current_axis_subscriber_ = nh.subscribe("/hippocampus/currentaxis", 1, &data_subscriber::CurrentAxisCallback,this);
    // add more subscribers here, as needed
    }
    //Get Position and Orientation from ROS
    void data_subscriber::PositionCallback(const geometry_msgs::PoseStamped::ConstPtr &pos) {
        data_subscriber::timeStamp = pos->header.stamp;
        //ROS_INFO("in callback function i heard %f:", pos->pose.position.x);
        data_subscriber::position = Vec3(pos->pose.position.x,pos->pose.position.y,pos->pose.position.z);
        data_subscriber::orientation = {pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w};
        //ROS_INFO("CAllBack Orient %f %f %f %f: ",orientation[0], orientation[1], orientation[2],orientation[3]  );
    }
    //Get Velocity from ROS
    void data_subscriber::VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &velo) {
        
        //ROS_INFO("in callback function velocity i heard %f:", velo->twist.linear.x);
        data_subscriber::velocity = Vec3(velo->twist.linear.x,velo->twist.linear.y,velo->twist.linear.z);
        
        
    }
    //Get Acceleration from ROS
    void data_subscriber::AccelerationCallback(const sensor_msgs::Imu::ConstPtr &accel) {
        
        //ROS_INFO("in callback function accel i heard %f:", accel->linear_acceleration.x);
        data_subscriber::acceleration = Vec3(accel->linear_acceleration.x,accel->linear_acceleration.y,accel->linear_acceleration.z);
        
        
    }
    void data_subscriber::AngularVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &angu){
       
       // ROS_INFO("in callback function angularvelo i heard %f:", angu->twist.angular.x);
        data_subscriber::angular_velocity = Vec3(angu->twist.angular.x,angu->twist.angular.y,angu->twist.angular.z);
    }
  void data_subscriber::CurrentAxisCallback(const mavros_msgs::HippocampusCurrentaxis::ConstPtr &axis){
       // ROS_INFO("in callback function currentaxis i heard %f:", axis->axis_x);
        data_subscriber::current_axis = Vec3(axis->axis_x, axis->axis_y, axis->axis_z);
    }
   
