/*!
 * Rapid trajectory generation for quadrocopters
 *
 * Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Nathan:
 * Reformatted
 * Fixed problems with GetOmega (TEST THIS)
 */
//modified by Christian-Hendrik Horst (TUHH)
#include <algorithm>
#include <limits>
#include <cerrno>
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"
#include "Quartic/quartic.h"
#include <chrono>
using namespace CommonMath;
using namespace RapidQuadrocopterTrajectoryGenerator;

RapidTrajectoryGenerator::RapidTrajectoryGenerator(const Vec3 x0, const Vec3 v0,
                                                   const Vec3 a0) {
  //initialise each axis:
  Reset();
  for (int i = 0; i < 3; i++)
    _axis[i].SetInitialState(x0[i], v0[i], a0[i]);
  
}

void RapidTrajectoryGenerator::SetGoalPosition(const Vec3 in) {
  for (unsigned i = 0; i < 3; i++)
    SetGoalPositionInAxis(i, in[i]);
}

void RapidTrajectoryGenerator::SetGoalVelocity(const Vec3 in) {
  for (int i = 0; i < 3; i++)
    SetGoalVelocityInAxis(i, in[i]);
}

void RapidTrajectoryGenerator::SetGoalAcceleration(const Vec3 in) {
  for (int i = 0; i < 3; i++)
    SetGoalAccelerationInAxis(i, in[i]);
}

void RapidTrajectoryGenerator::Reset(void) {
  for (int i = 0; i < 3; i++) {
    _axis[i].Reset();
  }
  _tf = 0;
}

//Generate the trajectory:
void RapidTrajectoryGenerator::Generate(const double timeToFinish) {
  _tf = timeToFinish;
  for (int i = 0; i < 3; i++) {
    _axis[i].GenerateTrajectory(_tf);
  }
}

RapidTrajectoryGenerator::InputFeasibilityResult RapidTrajectoryGenerator::CheckInputFeasibilitySection(
    double fminAllowed, double fmaxAllowed, double wmaxAllowed, double t1,
    double t2, double minTimeSection) {
  if (t2 - t1 < minTimeSection)
    return InputIndeterminable;
  //test the acceleration at the two limits:
  if (std::max(GetThrust(t1), GetThrust(t2)) > fmaxAllowed)
    return InputInfeasibleThrustHigh;
  if (std::min(GetThrust(t1), GetThrust(t2)) < fminAllowed)
    return InputInfeasibleThrustLow;

  double fminSqr = 0;
  double fmaxSqr = 0;
  double jmaxSqr = 0;

  //Test the limits of the box we're putting around the trajectory:
  for (int i = 0; i < 3; i++) {
    double amin, amax;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    //_axis[i].GetMinMaxAcc(amin, amax, t1, t2);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //std::cout << "Time difference AminMax Gen= " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    //std::cout << "Time difference AminMax Gen= " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
    //distance from zero thrust point in this axis
    //double v1 = 2.6 * amax + 5.4 * 0.9 ;  //left
    //double v2 = 2.6 * amax + 5.4 * 0.9;  //right
   
//Maximum of m*a + c*v
    //std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
    double roots[6];
    roots[0] = t1;
    roots[1] = t2;
    size_t rootCount;
    double c[5] = { 0, 0, 0, 0, 0 };
    c[0]=0;
    c[1]= (damping * (_axis[i].GetParamAlpha() / 24.0)) * 4.0;
    c[2]= (massparam * (_axis[i].GetParamAlpha()/6.0) + (damping *  (_axis[i].GetParamBeta()/6.0)) * 3.0) ;
    c[3]= (massparam * (_axis[i].GetParamBeta()/2.0) + (damping *  (_axis[i].GetParamGamma()/2.0)) * 2.0);
    c[4]= (massparam * (_axis[i].GetParamGamma()/1.0) + (damping *  (_axis[i].GetInitialAcceleration()/1.0)) * 1.0);
    
    rootCount = Quartic::solveP3(c[2] / c[1], c[3] / c[1], c[4] / c[1], roots);
   // ROS_INFO("Inside MaxCalculation1 %f  %f %f %f ", roots[2],roots[3],roots[4],roots[5]); 
    //ROS_INFO("Inside RootCount %lu ", rootCount); 
    if(roots[2]<0)roots[2]=0;
    //ROS_INFO("GetThrust : %f  v1 : %f ", GetThrust(roots[2]) ,v1);
    double v1 = GetThrust(roots[2]);
    double v2 = GetThrust(roots[2]);
    maximalThrust = v1;
    
    minimalThrust = GetThrust(roots[3]);
   // ROS_INFO("Max Min Thrust %f ", maximalThrust); 
   // ROS_INFO("Minimum Thrust Formula %f", std::min(maximalThrust,minimalThrust));
    //std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
    //std::cout << "Time difference Root Finder= " << std::chrono::duration_cast<std::chrono::microseconds>(end2 - begin2).count() << "[µs]" << std::endl;
    //std::cout << "Time difference Root Finder= " << std::chrono::duration_cast<std::chrono::nanoseconds> (end2 - begin2).count() << "[ns]" << std::endl;
    
//Maximum of m*j + c*a
    double roots2[6];
    roots2[0] = t1;
    roots2[1] = t2;
    size_t rootCount2;
    double c2[5] = { 0, 0, 0, 0, 0 };
    c2[0]= 0;
    c2[1]= 0;
    c2[2]= (damping * (_axis[i].GetParamAlpha()/2.0) );
    c2[3]= (massparam * (_axis[i].GetParamAlpha()/1.0) + (damping *  (_axis[i].GetParamBeta()/1.0)) * 1.0);
    c2[4]= (massparam * (_axis[i].GetParamBeta()/1.0) + (damping *  (_axis[i].GetParamGamma()/1.0)) * 1.0);

    //Vec3 rootsQuadratic = RapidTrajectoryGenerator::CalculateQuadraticExtrema(c2[2], c2[3],  c2[4]); 
    //ROS_INFO("Inside MaxCalculation2 %f  %f %f ", rootsQuadratic[0],rootsQuadratic[1],rootsQuadratic[2]);
    //double maximum1 = _axis[i].GetJerk(rootsQuadratic[1]) * massparam + _axis[i].GetAcceleration(rootsQuadratic[1]) * damping;
    //double maximum2 = _axis[i].GetJerk(rootsQuadratic[2]) * massparam + _axis[i].GetAcceleration(rootsQuadratic[2]) * damping;
   //double maximum3 = c[1]*pow(rootsQuadratic[1],3)+ c[2]*pow(rootsQuadratic[1],2)+ c[3] * pow(rootsQuadratic[1],1) +c[4];
   // double maximum4 = c[1]*pow(rootsQuadratic[2],3)+ c[2]*pow(rootsQuadratic[2],2)+ c[3] * pow(rootsQuadratic[2],1) +c[4];
  //  bodyrateBound = (std::max(maximum1,maximum2) / std::min(maximalThrust,minimalThrust));
   // ROS_INFO("MAXIMA1 %f  %f  ", maximum1,maximum2);
   // ROS_INFO("Maxima FDot %f:", std::max(maximum1,maximum2));
    //  ROS_INFO("BODY RATES %f :",bodyrateBound);
   // ROS_INFO("MAXIMA2 %f  %f  ", maximum3,maximum4);

    //   if (std::max(pow(v1, 2), pow(v2, 2)) > pow(fmaxAllowed, 2)
    //definitely infeasible:
   // if ((std::max(pow(v1, 2), pow(v2, 2)) > pow(fmaxAllowed, 2))||bodyrateBound > 3.0)
      if(std::max(pow(maximalThrust, 2), pow(maximalThrust, 2)) > pow(fmaxAllowed, 2))
      return InputInfeasibleThrustHigh;

    if (v1 * v2 < 0) {
      //sign of acceleration changes, so we've gone through zero
      fminSqr += 0;
    } else {
      fminSqr += pow(std::min(fabs(v1), fabs(v2)), 2);
    }

    fmaxSqr += pow(std::max(fabs(v1), fabs(v2)), 2);

    jmaxSqr += _axis[i].GetMaxJerkSquared(t1, t2);
  }

  double fmin = sqrt(fminSqr);
  double fmax = sqrt(fmaxSqr);
  double wBound;
  if (fminSqr > 1e-6)
    wBound = sqrt(jmaxSqr / fminSqr);  //the 1e-6 is a divide-by-zero protection
    
  else
    wBound = std::numeric_limits<double>::max();
  maxRate = wBound;
  //ROS_INFO("WBound : %f", wBound);
  //definitely infeasible:
  if (fmax < fminAllowed)
    return InputInfeasibleThrustLow;
  if (fmin > fmaxAllowed)
    return InputInfeasibleThrustHigh;

  //possibly infeasible:
  if (fmin < fminAllowed || fmax > fmaxAllowed || wBound > wmaxAllowed) {  //indeterminate: must check more closely:
    double tHalf = (t1 + t2) / 2;
    InputFeasibilityResult r1 = CheckInputFeasibilitySection(fminAllowed,
                                                             fmaxAllowed,
                                                             wmaxAllowed, t1,
                                                             tHalf,
                                                             minTimeSection);

    if (r1 == InputFeasible) {
      //continue with second half
      return CheckInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed,
                                          tHalf, t2, minTimeSection);
    }

    //first section is already infeasible, or indeterminate:
    return r1;
  }

  //definitely feasible:
  return InputFeasible;
}

RapidTrajectoryGenerator::InputFeasibilityResult RapidTrajectoryGenerator::CheckInputFeasibility(
    double fminAllowed, double fmaxAllowed, double wmaxAllowed,
    double minTimeSection) {
  //required thrust limits along trajectory
  double t1 = 0;
  double t2 = _tf;

  return CheckInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed, t1,
                                      t2, minTimeSection);
}

RapidTrajectoryGenerator::StateFeasibilityResult RapidTrajectoryGenerator::CheckPositionFeasibility(
    Vec3 boundaryPoint, Vec3 boundaryNormal) {
  //Ensure that the normal is a unit vector:
  boundaryNormal = boundaryNormal.GetUnitVector();

  

  //the coefficients of the quartic equation: x(t) = c[0]t**4 + c[1]*t**3 + c[2]*t**2 + c[3]*t + c[4]
  double c[5] = { 0, 0, 0, 0, 0 };

  for (int dim = 0; dim < 3; dim++) {
    c[0] += boundaryNormal[dim] * _axis[dim].GetParamAlpha() / 24.0;  //t**4
    c[1] += boundaryNormal[dim] * _axis[dim].GetParamBeta() / 6.0;  //t**3
    c[2] += boundaryNormal[dim] * _axis[dim].GetParamGamma() / 2.0;  //t**2
    c[3] += boundaryNormal[dim] * _axis[dim].GetInitialAcceleration();  //t
    c[4] += boundaryNormal[dim] * _axis[dim].GetInitialVelocity();  //1
  }

  //Solve the roots (we prepend the times 0 and tf):
  double roots[6];
  roots[0] = 0;
  roots[1] = _tf;

  size_t rootCount;
  if (fabs(c[0]) > 1e-6) {
    rootCount = Quartic::solve_quartic(c[1] / c[0], c[2] / c[0],
                                           c[3] / c[0], c[4] / c[0], roots);
  } else {
    rootCount = Quartic::solveP3(c[2] / c[1], c[3] / c[1], c[4] / c[1], roots);
  }

  for (unsigned i = 0; i < (rootCount + 2); i++) {
    //don't evaluate points outside the domain
    if (roots[i] < 0)
      continue;
    if (roots[i] > _tf)
      continue;

    if ((GetPosition(roots[i]) - boundaryPoint).Dot(boundaryNormal) <= 0) {
      //touching, or on the wrong side of, the boundary!
      return StateInfeasible;
    }
  }
  return StateFeasible;
}

Vec3 RapidTrajectoryGenerator::GetOmega(double t, double timeStep) const {
  //Calculates the body rates necessary at time t, to rotate the normal vector.
  //The result is coordinated in the world frame, i.e. would have to be rotated into a
  //body frame.
  const Vec3 n0 = GetNormalVector(t);
  const Vec3 n1 = GetNormalVector(t + timeStep);

  const Vec3 crossProd = n0.Cross(n1);  //direction of omega, in inertial axes
  if (crossProd.GetNorm2() <= 1e-6) {
    // Cross product failed
    return Vec3(0, 0, 0);
  } else {
    errno = 0;
    Vec3 n = crossProd.GetUnitVector();
    double angle = acos(n0.Dot(n1)) / timeStep;
    if (errno) {
      // either unit vector not defined or numeric issues cause dot product to fail
      return Vec3(0, 0, 0);
    } else {
      return angle * n;
    }
  }
}
Vec3 RapidTrajectoryGenerator::CalculateQuadraticExtrema(double a_coeff, double b_coeff, double c_coeff) const {
    Vec3 Result = Vec3(0, 0, 0);
    double  x1, x2, discriminant;
    discriminant = b_coeff * b_coeff - 4 * a_coeff * c_coeff;
    
    if (discriminant > 0) {
        x1 = (-b_coeff + sqrt(discriminant)) / (2*a_coeff);
        x2 = (-b_coeff - sqrt(discriminant)) / (2*a_coeff);
       // cout << "Roots are real and different." << endl;
        //cout << "x1 = " << x1 << endl;
        //cout << "x2 = " << x2 << endl;
    }
    
    else if (discriminant == 0) {
        //cout << "Roots are real and same." << endl;
        x1 = (-b_coeff + sqrt(discriminant)) / (2*a_coeff);
        x2 = x1;
        //cout << "x1 = x2 =" << x1 << endl;
    }

    else {
        double test = 0;
        x1 = 0;
        x2 = 0;
        //cout << "Roots are complex and different."  << endl;
        
    }
    Result[1]=x1;
    Result[2]=x2;
    return Result;
}