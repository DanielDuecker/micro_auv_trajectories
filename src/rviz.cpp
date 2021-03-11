#include "hippocampus_test/rviz.hpp"

rviz::rviz(ros::NodeHandle* nodehandle):nh(*nodehandle)
{

    initializePublisher(); 
}  
    void rviz::initializePublisher(){
       rviz_traject = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
       goal_visual = nh.advertise<visualization_msgs::Marker>("goal_visualization", 1);
       boat_position = nh.advertise<visualization_msgs::Marker>("boat_position", 1);
       single_obstacle = nh.advertise<visualization_msgs::Marker>("single_obstacle", 1);
       single_obstacle2 = nh.advertise<visualization_msgs::Marker>("single_obstacle2", 1);
       single_obstacle3 = nh.advertise<visualization_msgs::Marker>("single_obstacle3", 1);
       single_obstacle4 = nh.advertise<visualization_msgs::Marker>("single_obstacle4", 1);
       tank_walls = nh.advertise<visualization_msgs::MarkerArray>("tank_walls", 1);
       velo_info = nh.advertise<visualization_msgs::MarkerArray>("text_info", 1);
       data_info = nh.advertise<visualization_msgs::MarkerArray>("data_info", 1);
       rectangle = nh.advertise<visualization_msgs::Marker>("rectangle", 1);
    }

    void rviz::publishTrajectory(const RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator &ptr, const double tmr, const int coloroption){
      //  ROS_INFO("TEST");
        int trajectory_count = 1;
        int trajectory_markerpoints = 50;
        double r = 0.05 * 0.7;
        double tf = tmr;
        visualization_msgs::MarkerArray trajectory_marker; 
        trajectory_marker.markers.resize(trajectory_markerpoints);
        for(int i = 0; i < trajectory_markerpoints; i++){
            
            //visualization_msgs::Marker marker;
            trajectory_marker.markers[i].header.frame_id = "global_tank";
            trajectory_marker.markers[i].id = i;
        
            trajectory_marker.markers[i].type = visualization_msgs::Marker::SPHERE;;
            trajectory_marker.markers[i].action = visualization_msgs::Marker::ADD;;
            trajectory_marker.markers[i].scale.x = r;  
            trajectory_marker.markers[i].scale.y = r;
            trajectory_marker.markers[i].scale.z = r;
            trajectory_marker.markers[i].color.b = 1.0 - coloroption *0.8;
            trajectory_marker.markers[i].color.r = 1.0 + coloroption *4;
            trajectory_marker.markers[i].color.g = 1.0 - coloroption *0.8;

            trajectory_marker.markers[i].color.a = 1.0;  
            trajectory_marker.markers[i].pose.orientation.w = 1.0;
            CommonMath::Vec3 markerposition = ptr.GetPosition(((tf) / trajectory_markerpoints) * i);
            
            trajectory_marker.markers[i].pose.position.x = markerposition[0];  
            trajectory_marker.markers[i].pose.position.y = markerposition[1]; 
            trajectory_marker.markers[i].pose.position.z = markerposition[2];  
            //trajectory_marker.marker[i] = marker;
        }
        
            rviz_traject.publish(trajectory_marker);
        }
        
    void rviz::publishGoal(const Vec3 pose){
        visualization_msgs::Marker goal_pose;
        double r = 0.2;
        goal_pose.header.frame_id = "global_tank";
        
        goal_pose.id = 0;
        goal_pose.type =  goal_pose.CYLINDER;
        goal_pose.action =  goal_pose.ADD;
        goal_pose.scale.x = r;  
        goal_pose.scale.y = r;
        goal_pose.scale.z = 0.01;
        goal_pose.color.r = 1.0;
        goal_pose.color.g = 1.0;
        goal_pose.color.b = 0.6;
        goal_pose.color.a = 0.5 ; 
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.7071068;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 0.7071068;
        goal_pose.pose.position.x = pose[0]; 
        goal_pose.pose.position.y = pose[1]; 
        goal_pose.pose.position.z = pose[2];  
        goal_visual.publish(goal_pose);
    }
    void rviz::publishBoatPosition(const Vec3 position, const int counter){
        double r = 0.05 * 0.7;
        visualization_msgs::Marker boat_pose;
        boat_pose.header.frame_id = "global_tank";
        boat_pose.id = counter;
        boat_pose.type =  boat_pose.SPHERE;
        boat_pose.action =  boat_pose.ADD;
        boat_pose.scale.x = r;  
        boat_pose.scale.y = r;
        boat_pose.scale.z = r;
        boat_pose.color.g = 0.8;
        boat_pose.color.a = 0.5 ; 
        boat_pose.pose.orientation.w = 1.0;
        boat_pose.pose.position.x = position[0]; 
        boat_pose.pose.position.y = position[1]; 
        boat_pose.pose.position.z = position[2];  
        goal_visual.publish(boat_pose);
        
    }
    void rviz::publishVelocityText(const Vec3 velocity){
        visualization_msgs::MarkerArray marker; 
        marker.markers.resize(3);
        for(int i = 0; i < 3; i++){
            
            marker.markers[i].header.frame_id = "global_tank";
            marker.markers[i].header.stamp = ros::Time::now();
            marker.markers[i].ns = "basic_shapes";
            marker.markers[i].id = i;
            marker.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.markers[i].action = visualization_msgs::Marker::ADD;

            marker.markers[i].pose.position.x = 0.0 ;
            marker.markers[i].pose.position.y = 1.0;
            marker.markers[i].pose.position.z = 1.0-i*0.15;
            marker.markers[i].pose.orientation.x = 0.0;
            marker.markers[i].pose.orientation.y = 0.0;
            marker.markers[i].pose.orientation.z = 0.0;
            marker.markers[i].pose.orientation.w = 1.0;
           // if(i<4){
                std::string str = std::to_string(velocity[i]);
                marker.markers[i].text = str;
          //  }else{
             //   std::string str = std::to_string(timer);
               // marker.markers[i].text = str;
           // }

            marker.markers[i].scale.x = 0.3;
            marker.markers[i].scale.y = 0.3;
            marker.markers[i].scale.z = 0.1;

            marker.markers[i].color.r = 0.0f;
            marker.markers[i].color.g = 1.0f;
            marker.markers[i].color.b = 0.0f;
            marker.markers[i].color.a = 1.0;
        }
        velo_info.publish(marker);
        
    }
        void rviz::publishDataText(const Vec3 data){
        visualization_msgs::MarkerArray marker; 
        marker.markers.resize(3);
        for(int i = 0; i < 3; i++){
            
            marker.markers[i].header.frame_id = "global_tank";
            marker.markers[i].header.stamp = ros::Time::now();
            marker.markers[i].ns = "basic_shapes";
            marker.markers[i].id = i;
            marker.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.markers[i].action = visualization_msgs::Marker::ADD;

            marker.markers[i].pose.position.x = 0.0 ;
            marker.markers[i].pose.position.y = 2.0;
            marker.markers[i].pose.position.z = 1.0-i*0.15;
            marker.markers[i].pose.orientation.x = 0.0;
            marker.markers[i].pose.orientation.y = 0.0;
            marker.markers[i].pose.orientation.z = 0.0;
            marker.markers[i].pose.orientation.w = 1.0;
           // if(i<4){
                std::string str = std::to_string(data[i]);
                marker.markers[i].text = str;
          //  }else{
             //   std::string str = std::to_string(timer);
               // marker.markers[i].text = str;
           // }

            marker.markers[i].scale.x = 0.3;
            marker.markers[i].scale.y = 0.3;
            marker.markers[i].scale.z = 0.1;

            marker.markers[i].color.r = 0.0f;
            marker.markers[i].color.g = 1.0f;
            marker.markers[i].color.b = 0.0f;
            marker.markers[i].color.a = 1.0;
        }
        data_info.publish(marker);
        
    }
    void rviz::publishSingleObstacle(const Vec3 position, const double radius){
        double r = radius;
        
        visualization_msgs::Marker obs_pose;
        obs_pose.header.frame_id = "global_tank";
        obs_pose.id = 1;
        obs_pose.type =  obs_pose.SPHERE;
        obs_pose.action =  obs_pose.ADD;
        obs_pose.scale.x = 2*r;  
        obs_pose.scale.y = 2*r;
        obs_pose.scale.z = 2*r;
        obs_pose.color.r = 0.5;
        obs_pose.color.a = 0.7 ; 
        obs_pose.pose.orientation.w = 1.0;
        obs_pose.pose.position.x = position[0]; 
        obs_pose.pose.position.y = position[1]; 
        obs_pose.pose.position.z = position[2];  
        single_obstacle.publish(obs_pose);
        
    }
        void rviz::publishSingleObstacle2(const Vec3 position, const double radius){
        double r = radius;
        
        visualization_msgs::Marker obs_pose;
        obs_pose.header.frame_id = "global_tank";
        obs_pose.id = 1;
        obs_pose.type =  obs_pose.SPHERE;
        obs_pose.action =  obs_pose.ADD;
        obs_pose.scale.x = 2*r;  
        obs_pose.scale.y = 2*r;
        obs_pose.scale.z = 2*r;
        obs_pose.color.r = 0.5;
        obs_pose.color.a = 0.7 ; 
        obs_pose.pose.orientation.w = 1.0;
        obs_pose.pose.position.x = position[0]; 
        obs_pose.pose.position.y = position[1]; 
        obs_pose.pose.position.z = position[2];  
        single_obstacle2.publish(obs_pose);
        
    }
    void rviz::publishSingleObstacle3(const Vec3 position, const double radius){
        double r = radius;
        
        visualization_msgs::Marker obs_pose;
        obs_pose.header.frame_id = "global_tank";
        obs_pose.id = 1;
        obs_pose.type =  obs_pose.SPHERE;
        obs_pose.action =  obs_pose.ADD;
        obs_pose.scale.x = 2*r;  
        obs_pose.scale.y = 2*r;
        obs_pose.scale.z = 2*r;
        obs_pose.color.r = 0.5;
        obs_pose.color.a = 0.7 ; 
        obs_pose.pose.orientation.w = 1.0;
        obs_pose.pose.position.x = position[0]; 
        obs_pose.pose.position.y = position[1]; 
        obs_pose.pose.position.z = position[2];  
        single_obstacle3.publish(obs_pose);
        
    }
    void rviz::publishSingleObstacle4(const Vec3 position, const double radius){
        double r = radius;
        
        visualization_msgs::Marker obs_pose;
        obs_pose.header.frame_id = "global_tank";
        obs_pose.id = 1;
        obs_pose.type =  obs_pose.SPHERE;
        obs_pose.action =  obs_pose.ADD;
        obs_pose.scale.x = 2*r;  
        obs_pose.scale.y = 2*r;
        obs_pose.scale.z = 2*r;
        obs_pose.color.b = 0.5;
        obs_pose.color.g = 0.5;
        obs_pose.color.a = 0.5 ; 
        obs_pose.pose.orientation.w = 1.0;
        obs_pose.pose.position.x = position[0]; 
        obs_pose.pose.position.y = position[1]; 
        obs_pose.pose.position.z = position[2];  
        single_obstacle4.publish(obs_pose);
        
    }
    void rviz::publishRectangle(const Vec3 position, const double radius){
        double r = radius;
        
        visualization_msgs::Marker obs_pose;
        obs_pose.header.frame_id = "global_tank";
        obs_pose.id = 1;
        obs_pose.type =  obs_pose.CUBE;
        obs_pose.action =  obs_pose.ADD;
        obs_pose.scale.x = 0.35;  
        obs_pose.scale.y = 0.35;
        obs_pose.scale.z = 1.5;
        obs_pose.color.r = 0.5;
        obs_pose.color.a = 0.7 ; 
        obs_pose.pose.orientation.w = 1.0;
        obs_pose.pose.position.x = position[0]; 
        obs_pose.pose.position.y = position[1]; 
        obs_pose.pose.position.z = 0.75;//position[2];  
        rectangle.publish(obs_pose);
        
    }
   void rviz::publishWalls(){
       
   
       visualization_msgs::MarkerArray all_tank_walls; 
       all_tank_walls.markers.resize(4);
       //visualization_msgs::Marker wall1,wall2,wall3,wall4;
       
       all_tank_walls.markers[0].header.frame_id = "global_tank";
       all_tank_walls.markers[0].id = 0;
       all_tank_walls.markers[0].type =  visualization_msgs::Marker::CUBE;
       all_tank_walls.markers[0].action =  visualization_msgs::Marker::ADD;
       
           all_tank_walls.markers[1].header.frame_id = "global_tank";
        all_tank_walls.markers[1].id = 1;
        all_tank_walls.markers[1].type =  visualization_msgs::Marker::CUBE;
        all_tank_walls.markers[1].action =  visualization_msgs::Marker::ADD;
       
           all_tank_walls.markers[2].header.frame_id = "global_tank";
        all_tank_walls.markers[2].id = 2;
        all_tank_walls.markers[2].type =  visualization_msgs::Marker::CUBE;
        all_tank_walls.markers[2].action =  visualization_msgs::Marker::ADD;
       
           all_tank_walls.markers[3].header.frame_id = "global_tank";
        all_tank_walls.markers[3].id = 3;
        all_tank_walls.markers[3].type =  visualization_msgs::Marker::CUBE;
        all_tank_walls.markers[3].action =  visualization_msgs::Marker::ADD;
    //wall 1--------------------------------------------
        all_tank_walls.markers[0].scale.x = 0.005  ;
        all_tank_walls.markers[0].scale.y = 1.5;
        all_tank_walls.markers[0].scale.z = 1.5;
       
        all_tank_walls.markers[0].color.r = 0.0 + 0.5;
        all_tank_walls.markers[0].color.g = 1.0;
        all_tank_walls.markers[0].color.b = 0.6;
        all_tank_walls.markers[0].color.a = 0.1;    

        all_tank_walls.markers[0].pose.orientation.x = 0.0;
        all_tank_walls.markers[0].pose.orientation.y = 0.0;
        all_tank_walls.markers[0].pose.orientation.z = 0.0;
        all_tank_walls.markers[0].pose.orientation.w = 1.0;
        all_tank_walls.markers[0].pose.position.x = 0.0; 
        all_tank_walls.markers[0].pose.position.y = 0.75 ;
        all_tank_walls.markers[0].pose.position.z = 0.75 ; 
    //wall 2--------------------------------------------
       all_tank_walls.markers[1].scale.x = 0.005  ;
       all_tank_walls.markers[1].scale.y = 1.5;
       all_tank_walls.markers[1].scale.z = 1.5;
       
       all_tank_walls.markers[1].color.r = 0.0 + 1.5;
       all_tank_walls.markers[1].color.g = 1.0;
       all_tank_walls.markers[1].color.b = 0.6;
       all_tank_walls.markers[1].color.a = 0.1;   

       all_tank_walls.markers[1].pose.orientation.x = 0.0;
       all_tank_walls.markers[1].pose.orientation.y = 0.0;
       all_tank_walls.markers[1].pose.orientation.z = 0.0;
       all_tank_walls.markers[1].pose.orientation.w = 1.0;
       all_tank_walls.markers[1].pose.position.x = 3.5; 
       all_tank_walls.markers[1].pose.position.y = 0.75 ;
       all_tank_walls.markers[1].pose.position.z = 0.75 ; 
    //wall 3--------------------------------------------2,2,0.5,0.707,0.707
       all_tank_walls.markers[2].scale.x = 0.005  ;
       all_tank_walls.markers[2].scale.y = 3.5;
       all_tank_walls.markers[2].scale.z = 1.5;
       
       all_tank_walls.markers[2].color.r = 0.0 + 2.0;
       all_tank_walls.markers[2].color.g = 1.0;
       all_tank_walls.markers[2].color.b = 0.6;
       all_tank_walls.markers[2].color.a = 0.1;   

       all_tank_walls.markers[2].pose.orientation.x = 0.0;
       all_tank_walls.markers[2].pose.orientation.y = 0.0;
       all_tank_walls.markers[2].pose.orientation.z = 0.707;
       all_tank_walls.markers[2].pose.orientation.w = 0.707;
       all_tank_walls.markers[2].pose.position.x = 1.75; 
       all_tank_walls.markers[2].pose.position.y = 1.5 ;
       all_tank_walls.markers[2].pose.position.z = 0.75 ; 
    //wall 4--------------------------------------------2,0,0.5,0.707,0.707
       all_tank_walls.markers[3].scale.x = 0.005  ;
       all_tank_walls.markers[3].scale.y = 3.5;
       all_tank_walls.markers[3].scale.z = 1.5;
       
       all_tank_walls.markers[3].color.r = 0.0 + 2.5;
       all_tank_walls.markers[3].color.g = 1.0;
       all_tank_walls.markers[3].color.b = 0.6;
       all_tank_walls.markers[3].color.a = 0.1;   

       all_tank_walls.markers[3].pose.orientation.x = 0.0;
       all_tank_walls.markers[3].pose.orientation.y = 0.0;
       all_tank_walls.markers[3].pose.orientation.z = 0.707;
       all_tank_walls.markers[3].pose.orientation.w = 0.707;
       all_tank_walls.markers[3].pose.position.x = 1.75; 
       all_tank_walls.markers[3].pose.position.y = 0.0 ;
       all_tank_walls.markers[3].pose.position.z = 0.75 ; 
       
       tank_walls.publish(all_tank_walls);
       
       
      
       
   }
   
   