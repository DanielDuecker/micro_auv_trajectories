#ifndef TRAJECTORYCREATOR_HPP
#define TRAJECTORYCREATOR_HPP
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"
#include "RapidCollisionDetection/CollisionChecker.hpp"
#include "CommonMath/Vec3.hpp"
#include <list> 
#include <algorithm>
#include <chrono>
#include "CommonMath/RectPrism.hpp"
#include "CommonMath/Rotation.hpp"
#include "CommonMath/ConvexObj.hpp"
#include "CommonMath/Sphere.hpp"
using namespace RapidQuadrocopterTrajectoryGenerator;
using namespace std;
using namespace RapidCollisionChecker;

class TrajectoryCreator
{
public:
    TrajectoryCreator();
    RapidTrajectoryGenerator GenerateTrajectories(const CommonMath::Vec3 position, const CommonMath::Vec3 velocity, const CommonMath::Vec3 acceleration, const int iterations, const double timeduration);
    void SetNewPos(const CommonMath::Vec3 in);
    void SetNewVel(const CommonMath::Vec3 in);
    void SetNewAccel(const CommonMath::Vec3 in);
    void setMode(const int modus);
    void DeleteTrajectoryList() { TrajectoryCreator::trajectory_list.clear(); };
    int getFeasibleCount(void) const{return feasibleCount;};
    
private:
    CommonMath::Vec3 goal_pos;
    CommonMath::Vec3 goal_vel;
    CommonMath::Vec3 goal_accel;
    std::vector<RapidTrajectoryGenerator> trajectory_list;
    int feasibleCount;
    int mode;
    int ChooseBestTrajectory();
};

#endif // TRAJECTORYCREATOR_HPP
