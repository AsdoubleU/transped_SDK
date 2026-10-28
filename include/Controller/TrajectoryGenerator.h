#ifndef TRAJECTORYGENERATOR_H_
#define TRAJECTORYGENERATOR_H_

#include <iostream>
#include <cmath>
#include "Eigen/Dense"

// Trajectory Generator (transped_mujoco 형식, DoF는 motors.yaml 기준 가변)
class TrajectoryGenerator
{
public:
    TrajectoryGenerator() {};
    virtual ~TrajectoryGenerator() {};

    void init(double dt, size_t dof);

    // qi -> qf 를 period 동안 cosine 보간. 호출마다 dt 만큼 진행.
    void SetSinusoidalTrajectory(const Eigen::VectorXd& qf, const Eigen::VectorXd& qi, double period);
    void ResetTraj() { isEnd = false; t_ = 0.; first_start = true; }

    const Eigen::VectorXd& GetRefVar() const { return ref_var_; }
    const Eigen::VectorXd& GetRefVarDot() const { return ref_vardot_; }
    const Eigen::VectorXd& GetRefVarDDot() const { return ref_varddot_; }
    bool isEnd = false, first_start = true;

private:
    Eigen::VectorXd ref_var_, ref_vardot_, ref_varddot_, distance;

    double t_ = 0., pi_ = M_PI, dt_ = 0., vel_scale = 0., acc_scale = 0.;
};

#endif
