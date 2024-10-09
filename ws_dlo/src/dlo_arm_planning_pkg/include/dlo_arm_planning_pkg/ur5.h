#ifndef DUAL_UR_UR5_H
#define DUAL_UR_UR5_H

#include <math.h>
#include <stdio.h>
#include <memory>
#include <vector>
#include <random>

#include <Eigen/Core>
#include <Eigen/Dense> 
#include <Eigen/Geometry>

// These kinematics find the tranfrom from the base link to the end effector.
// Though the raw D-H parameters specify a transform from the 0th link to the 6th link,
// offset transforms are specified in this formulation.
// To work with the raw D-H kinematics, use the inverses of the transforms below.

// Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1

namespace dual_ur{

class UR5{
public:
  typedef std::shared_ptr<UR5> Ptr;

    UR5(){}

    // @param q       The 6 joint values 
    // @param T       The 4x4 end effector pose in row-major ordering
    void forward(const double* q, double* T);

    // @param q       The 6 joint values 
    // @param Ti      The 4x4 link i pose in row-major ordering. If NULL, nothing is stored.
    void forward_all(const double* q, double* T1, double* T2, double* T3, 
                                    double* T4, double* T5, double* T6);

    // @param T       The 4x4 end effector pose in row-major ordering
    // @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
    // @param q6_des  An optional parameter which designates what the q6 value should take
    //                in case of an infinite solution on that joint.
    // @return        Number of solutions found (maximum of 8)
    int inverse(const double* T, double* q_sols, double q6_des=0.0);

    

    int inverse(
      const Eigen::Isometry3d &pose,
      std::vector<std::vector<double> > &q_solutions,
      double q6_des=0.0
    );



private:
    const double d1 =  0.089159;
    const double a2 = -0.42500;
    const double a3 = -0.39225;
    const double d4 =  0.10915;
    const double d5 =  0.09465;
    const double d6 =  0.0823;

    const double ZERO_THRESH = 0.00000001;
    const double PI = M_PI;



private:
  static int SIGN(double x) {
      return (x > 0) - (x < 0);
  }

};

} // namespace

#endif