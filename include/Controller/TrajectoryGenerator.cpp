#include "TrajectoryGenerator.h"

void TrajectoryGenerator::init(double dt, size_t dof)
{
    dt_ = dt;
    ref_var_ = Eigen::VectorXd::Zero(dof);
    ref_vardot_ = Eigen::VectorXd::Zero(dof);
    ref_varddot_ = Eigen::VectorXd::Zero(dof);
    distance = Eigen::VectorXd::Zero(dof);
}

void TrajectoryGenerator::SetSinusoidalTrajectory(const Eigen::VectorXd& qf, const Eigen::VectorXd& qi, double period)
{
    if(period <= 0){
        std::cerr<<"[Trajectory Generator] Invalid Period!"<<std::endl;
        return;
    }

    if (t_ >= period) {
        t_ = period;
        isEnd = true;
        ref_var_ = qf;
        ref_vardot_.setZero();
        ref_varddot_.setZero();
        return;
    }

    if(first_start == true){
        distance = (qf - qi);
        vel_scale = 0.5 * (pi_ / period);
        acc_scale = 0.5 * (pi_ / period)*(pi_ / period);
        first_start = false;
    }

    double phase = pi_ * t_ / period, cosine = cos(phase), sine = sin(phase);

    ref_var_.noalias() = distance*0.5*(1-cosine) + qi;
    ref_vardot_.noalias() = distance*vel_scale*sine;
    ref_varddot_.noalias() = distance*acc_scale*cosine;

    t_ += dt_;
}
