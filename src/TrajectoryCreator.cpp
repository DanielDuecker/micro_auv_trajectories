#include "hippocampus_test/TrajectoryCreator.hpp"
#include <random>
#include <fstream>
#include <iostream>
using std::ofstream;
using namespace std;
using namespace CommonMath;
TrajectoryCreator::TrajectoryCreator()
{
    
    TrajectoryCreator::mode =0;
}
RapidTrajectoryGenerator TrajectoryCreator::GenerateTrajectories(const Vec3 position, const Vec3 velocity, const Vec3 acceleration, const int iterations, const double timeduration){
    double fmin = 0.0;//[m/s**2]
    double fmax = 8.0;//[m/s**2]
    double wmax = 3;//[rad/s]
    double minTimeSec = 0.02;//[s]
    int inputfeasibility = 0;
    int positionfeasibility1 =0;
    int positionfeasibility2 =0;
    int wallCounter=0;
    int obstacleCounter=0;
    int trajCounter=0;
    ////////////////left////////////////
    CommonMath::Vec3 leftWallpoint = CommonMath::Vec3(2.0, 0.05,0.5);
    CommonMath::Vec3 leftWallnormal = CommonMath::Vec3(0,1,0);
    /////////////////right//////////////
    CommonMath::Vec3 rightWallpoint = CommonMath::Vec3(2.0, 1.6,0.5);
    CommonMath::Vec3 rightWallnormal = CommonMath::Vec3(0,-1,0);
    mt19937 gen(0); 
    
    
///////////////////////////////SINGLE TRAJECTORY///////////////////////////////////////////////////
if (TrajectoryCreator::mode==1){
   // ROS_INFO("Inside Single");
    uniform_real_distribution<> randPosX(goal_pos[0] - 0.0, goal_pos[0] + 0.0);
    uniform_real_distribution<> randPosY(goal_pos[1] - 0.0, goal_pos[1] + 0.0);
    uniform_real_distribution<> randPosZ(goal_pos[2] - 0.00, goal_pos[2] + 0.00);
    
    uniform_real_distribution<> randVelX(goal_vel[0] - 0.00, goal_vel[0] + 0.000);
    uniform_real_distribution<> randVelY(goal_vel[1] - 0.00, goal_vel[1] + 0.000);
    uniform_real_distribution<> randVelZ(goal_vel[2] - 0.0, goal_vel[2] + 0.0);
    
    uniform_real_distribution<> randAccX(goal_accel[0] - 0.0, goal_accel[0] + 0.00);
    uniform_real_distribution<> randAccY(goal_accel[1] - 0.0, goal_accel[1] + 0.00);
    uniform_real_distribution<> randAccZ(goal_accel[2] - 0.0, goal_accel[2] + 0.00);
    //Time Sampling+
    double upperTimeBound = timeduration + 0.0;
    double lowerTimeBound = timeduration - 0.0;
    ////////////////////////////////////////////////////////////////////////////
    if(lowerTimeBound <= 0.3) lowerTimeBound = 0.3;
    uniform_real_distribution<> totaltime(lowerTimeBound , upperTimeBound);
   // ofstream outdata; 
    //outdata.open("comptime_positionfeasible.txt",std::ios_base::app);
 

    
    for(int i = 0; i < iterations; i++)
    {   
    
        CommonMath::Vec3 posf =   Vec3(randPosX(gen), randPosY(gen) , randPosZ(gen)); //position
        CommonMath::Vec3 velf =Vec3(randVelX(gen), randVelY(gen), randVelZ(gen)); //velocity
        //CommonMath::Vec3 accf = Vec3(randAccX(gen), randAccY(gen), randAccZ(gen)); //acceleration
        CommonMath::Vec3 accf = Vec3(0.0, 0.0,0.0);
        RapidTrajectoryGenerator traj(position, velocity, acceleration);
        traj.SetGoalPosition(posf);
        traj.SetGoalVelocity(velf);
        traj.SetGoalAcceleration(accf);
        traj.Generate(totaltime(gen));
        inputfeasibility =    traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec);
        //positionfeasibility = traj.CheckPositionFeasibility(Vec3(0.0, 0.0,0.0),Vec3(0.0, 0.0,1.0));

        if(inputfeasibility==0){
            TrajectoryCreator::trajectory_list.push_back(traj);
        }
       
    }
}  
////////////////////////////////MULTI TRAJECTORY///////////////////////////////////////////////////
if (TrajectoryCreator::mode==2){
   // ROS_INFO("Inside Multi");
    uniform_real_distribution<> randPosX(goal_pos[0] - 0.10, goal_pos[0] + 0.10);
    uniform_real_distribution<> randPosY(goal_pos[1] - 0.10, goal_pos[1] + 0.10);
    uniform_real_distribution<> randPosZ(goal_pos[2] - 0.05, goal_pos[2] + 0.05);
    
    uniform_real_distribution<> randVelX(goal_vel[0] - 0.05, goal_vel[0] + 0.05);
    uniform_real_distribution<> randVelY(goal_vel[1] - 0.05, goal_vel[1] + 0.05);
    uniform_real_distribution<> randVelZ(goal_vel[2] - 0.0, goal_vel[2] + 0.0);
    
    uniform_real_distribution<> randAccX(goal_accel[0] - 0.0, goal_accel[0] + 0.00);
    uniform_real_distribution<> randAccY(goal_accel[1] - 0.0, goal_accel[1] + 0.00);
    uniform_real_distribution<> randAccZ(goal_accel[2] - 0.0, goal_accel[2] + 0.00);
    //Time Sampling+
    double upperTimeBound = timeduration + 1.0;
    double lowerTimeBound = timeduration - 0.7;
    ////////////////////////////////////////////////////////////////////////////
    if(lowerTimeBound <= 0.3) lowerTimeBound = 0.3;
    uniform_real_distribution<> totaltime(lowerTimeBound , upperTimeBound);
   // ofstream outdata; 
    //outdata.open("comptime_positionfeasible.txt",std::ios_base::app);
 

    
    for(int i = 0; i < iterations; i++)
    {   
    
        CommonMath::Vec3 posf =   Vec3(randPosX(gen), randPosY(gen) , randPosZ(gen)); //position
        CommonMath::Vec3 velf =Vec3(randVelX(gen), randVelY(gen), randVelZ(gen)); //velocity
        //CommonMath::Vec3 accf = Vec3(randAccX(gen), randAccY(gen), randAccZ(gen)); //acceleration
        CommonMath::Vec3 accf = Vec3(0.0, 0.0,0.0);
        RapidTrajectoryGenerator traj(position, velocity, acceleration);
        traj.SetGoalPosition(posf);
        traj.SetGoalVelocity(velf);
        traj.SetGoalAcceleration(accf);
        traj.Generate(totaltime(gen));
        inputfeasibility =    traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec);
        positionfeasibility1 = traj.CheckPositionFeasibility(leftWallpoint,leftWallnormal);
  
        if(inputfeasibility==0 && positionfeasibility1==0){
            TrajectoryCreator::trajectory_list.push_back(traj);
        }
    
    }
     TrajectoryCreator::feasibleCount = trajectory_list.size();
    // ROS_INFO("LENGTH : %i", trajectory_list.size()); 
}
////////////////////////////////OBSTACLEAVOIDANCE///////////////////////////////////////////////////
if (TrajectoryCreator::mode==3){
        
   // ROS_INFO("Inside Obstacle");
    uniform_real_distribution<> randPosX(goal_pos[0] - 0.15, goal_pos[0] + 0.15);
    uniform_real_distribution<> randPosY(goal_pos[1] - 0.15, goal_pos[1] + 0.15);
    uniform_real_distribution<> randPosZ(goal_pos[2] - 0.0, goal_pos[2] + 0.0);
    
    uniform_real_distribution<> randVelX(goal_vel[0] - 0.0, goal_vel[0] + 0.0);
    uniform_real_distribution<> randVelY(goal_vel[1] - 0.1, goal_vel[1] + 0.1);
    uniform_real_distribution<> randVelZ(goal_vel[2] - 0.0, goal_vel[2] + 0.0);
    
    uniform_real_distribution<> randAccX(goal_accel[0] - 0.0, goal_accel[0] + 0.00);
    uniform_real_distribution<> randAccY(goal_accel[1] - 0.0, goal_accel[1] + 0.00);
    uniform_real_distribution<> randAccZ(goal_accel[2] - 0.0, goal_accel[2] + 0.00);
    //Time Sampling+
    double upperTimeBound = timeduration + 3.0;
    double lowerTimeBound = timeduration - 0.5;
    ////////////////////////////////////////////////////////////////////////////
    if(lowerTimeBound <= 0.3) lowerTimeBound = 0.3;
    uniform_real_distribution<> totaltime(lowerTimeBound , upperTimeBound);
   // ofstream outdata; 
    //outdata.open("comptime_positionfeasible.txt",std::ios_base::app);
 

    
    for(int i = 0; i < iterations; i++)
    {   
    
        CommonMath::Vec3 posf =   Vec3(randPosX(gen), randPosY(gen) , randPosZ(gen)); //position
        CommonMath::Vec3 velf =Vec3(randVelX(gen), randVelY(gen), randVelZ(gen)); //velocity
        //CommonMath::Vec3 accf = Vec3(randAccX(gen), randAccY(gen), randAccZ(gen)); //acceleration
        CommonMath::Vec3 accf = Vec3(0.0, 0.0,0.0);
        RapidTrajectoryGenerator traj(position, velocity, acceleration);
        traj.SetGoalPosition(posf);
        traj.SetGoalVelocity(velf);
        traj.SetGoalAcceleration(accf);
        traj.Generate(totaltime(gen));
        inputfeasibility =    traj.CheckInputFeasibility(fmin,fmax,wmax,minTimeSec);
        positionfeasibility1 = traj.CheckPositionFeasibility(leftWallpoint,leftWallnormal);
        positionfeasibility2 = traj.CheckPositionFeasibility(rightWallpoint,rightWallnormal);
        Vec3 obsPos(1.7, 0.75, 0.5);
     //   Vec3 obsPos2(1.7, 0.75, 0.8);
       // Vec3 obsPos3(1.7, 0.75, 1.2);
     //  Vec3 obsPos(1.44, 0.68, 0.5);
       // Vec3 obsPos(1.6, 1.4, 0.5);
       // Vec3 obsPos2(1.3, 0.8, 0.5);
       // Vec3 obsPos3(1.6, 0.3, 0.5);
       
        double obsRadius = 0.25;
        
        Vec3 prismPosition(1.7, 0.75, 0.0);
        Vec3 prismLength(0.35, 0.35, 1.5);
        Rotation prismRot (1.0,0.0,0.0,0.0);
        shared_ptr<ConvexObj> prism = make_shared < RectPrism > (prismPosition, prismLength,prismRot);
     //   double obsRadius2 = 0.25;
        shared_ptr<ConvexObj> obstacle = make_shared < Sphere> (obsPos, obsRadius);
     //   shared_ptr<ConvexObj> obstacle2 = make_shared < Sphere> (obsPos2, obsRadius);
       // shared_ptr<ConvexObj> obstacle3 = make_shared < Sphere> (obsPos3, obsRadius);
       // shared_ptr<ConvexObj> obstacle2 = make_shared < Sphere> (obsPos2, obsRadius);
      //  shared_ptr<ConvexObj> obstacle3 = make_shared < Sphere> (obsPos2, obsRadius);
        CollisionChecker checker(traj.GetTrajectory());
        
        CollisionChecker::CollisionResult stateFeas = checker.CollisionCheck(prism, 0.02);
        
        //if(position[0] < 2.0){
           // CollisionChecker::CollisionResult stateFeas = checker.CollisionCheck(obstacle2, 0.02);
          //  }
        
      //  CollisionChecker::CollisionResult stateFeas2 = checker.CollisionCheck(obstacle2, 0.04);
       // CollisionChecker::CollisionResult stateFeas3 = checker.CollisionCheck(obstacle3, 0.04);
        
        if(inputfeasibility==0 && positionfeasibility1==0 && positionfeasibility2==0  ){
            if(stateFeas == 0 ){
            // if(stateFeas2 == 0 ){
              //  if(stateFeas3 == 0 ){
                    TrajectoryCreator::trajectory_list.push_back(traj);
            //}
              //  }
            }
        }
       
    }
}    
    
    
    
    
    /////////////////////////////////////////////////////////////////////////////////////
   // ROS_INFO("Length of Traj List : %ld", trajectory_list.size() ); 
    int index =  TrajectoryCreator::ChooseBestTrajectory();   

    return trajectory_list[index];
    
    
    
    
    
}





////////////////////////////////////////////////////////////////////////////////////////////////////s
int TrajectoryCreator::ChooseBestTrajectory(){
    std::vector<double> cost_list;
    std::vector<double> duration_list;
    for(int k = 0; k < TrajectoryCreator::trajectory_list.size(); k++) {
        //cost_list.push_back( trajectory_list[k].GetCost());
       // duration_list.push_back( trajectory_list[k].GetCost());
        //printf("TimeOfTraject %f:", trajectory_list[k].GetDuration());
        duration_list.push_back( trajectory_list[k].GetDuration());
    }
    return std::min_element(duration_list.begin(),duration_list.end()) - duration_list.begin();
}

void TrajectoryCreator::SetNewPos(const CommonMath::Vec3 in)
{   
     TrajectoryCreator::goal_pos = in;
}
void TrajectoryCreator::SetNewVel(const CommonMath::Vec3 in)
{   
     TrajectoryCreator::goal_vel = in;
}
void TrajectoryCreator::SetNewAccel(const CommonMath:: Vec3 in)
{   
     TrajectoryCreator::goal_accel = in;  
}
void TrajectoryCreator::setMode(const int modus){
    TrajectoryCreator::mode = modus;
}