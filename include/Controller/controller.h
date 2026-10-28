#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <vector>
#include <iostream>
#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"

#include "TrajectoryGenerator.h"

#define RAD2DEG 57.2957802
#define DEG2RAD 0.0174533

enum ControlMode
{
    INIT,
    HOMING,
    FINISH
};

// 관절별 명령 프로토콜 (motors.yaml control_mode 에서 결정)
enum CommandType
{
    CMD_REFERENCE, // impedance / position / velocity : 궤적(q, qd)을 드라이브로 보내고 PD는 드라이브가 계산
    CMD_TORQUE     // torque(CST) : PC에서 kp*(q_ref-q) + kd*(qd_ref-qd) 계산 후 토크 전송
};

enum TargetMode : short
{
    POSITION,
    VELOCITY,
    TORQUE
};

// transped 전체 관절 수 (transped_mujoco robot_info.yaml joint_names 순서)
//    0 LSY   1 LSP   2 LEP   3 LWP  |  4 RSY   5 RSP   6 REP   7 RWP
//    8 LHP   9 LHR  10 LTY  11 LKP  12 LAP  13 LAR  |  14 RHP  15 RHR  16 RTY  17 RKP  18 RAP  19 RAR
constexpr size_t ROBOT_DOF = 20;

struct JointCommands
{
    std::vector<double> q_target;
    std::vector<double> qd_target;
    std::vector<double> torque_target;
};

class Controller
{

public:
    Controller(double frequency, const YAML::Node& config, const std::vector<CommandType>& command_types,
               const std::vector<int>& joint_indices);
    virtual ~Controller() {}

    double GetControlFrequency() { return control_frequency; }
    ControlMode GetControlMode() { return controlmode; }
    TargetMode GetTargetMode() { return target_mode; }

    // 유효한 명령을 내고 있는 상태인지 (INIT / FINISH 에서는 false -> 드라이브 disable)
    bool IsCommandValid() { return controlmode != INIT && controlmode != FINISH; }

    void command(bool flag);
    void sensor_callback(const std::vector<double>& q, const std::vector<double>& dq);
    const JointCommands& send_commands_to_robot();
    void finish(const char* reason);

    void print_status();

protected:
    size_t dof;
    double control_frequency, control_period;
    double control_time = 0.;

    ControlMode controlmode = INIT;
    bool Recieved_Mode[FINISH + 1] = {false};
    bool sensor_received = false;

    std::vector<CommandType> command_types_;
    TargetMode target_mode = POSITION;

    Eigen::VectorXd q_, dq_;
    Eigen::VectorXd torque_;
    Eigen::VectorXd ref_q_, ref_dq_;
    Eigen::VectorXd q_target, qdot_target;  // send_commands_to_robot 에서 전송할 위치 / 속도 명령
    Eigen::VectorXd target_pos, leg_init_pos;
    Eigen::VectorXd kp, kd, tau_max;

    // 등록된 각 관절의 ROBOT_DOF 순서 위치 (motors.yaml joint_index)
    std::vector<int> joint_indices_;
    // ROBOT_DOF 크기 자세에서 등록된 관절 값만 골라낸다.
    Eigen::VectorXd SelectJoints(const Eigen::VectorXd& pose) const;

    size_t homing_count = 0;
    double period = 0.;

    // config
    double homing_max_distance;
    double max_tracking_error;

    TrajectoryGenerator traj;
    JointCommands commands_;
};

#endif
