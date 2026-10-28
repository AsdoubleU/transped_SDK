#include "controller.h"

#include <algorithm>
#include <stdexcept>

namespace {

// yaml 값이 스칼라면 모든 관절에 같은 값, 리스트면 관절 수와 같아야 한다.
Eigen::VectorXd jointParam(const YAML::Node& node, size_t dof, double default_value, const char* name)
{
    Eigen::VectorXd v = Eigen::VectorXd::Constant(dof, default_value);
    if (!node) return v;
    if (node.IsScalar()) {
        v.setConstant(node.as<double>());
        return v;
    }
    const auto values = node.as<std::vector<double>>();
    if (values.size() != dof)
        throw std::runtime_error(std::string(name) + " size must be 1 or " + std::to_string(dof));
    for (size_t i = 0; i < dof; ++i) v(i) = values[i];
    return v;
}

} // namespace

Controller::Controller(double frequency, const YAML::Node& config, const std::vector<CommandType>& command_types,
                       const std::vector<int>& joint_indices):
dof(command_types.size()), command_types_(command_types), joint_indices_(joint_indices)
{
    control_frequency = frequency;
    control_period = 1./frequency;

    if (joint_indices_.size() != dof)
        throw std::runtime_error("joint_indices size must be " + std::to_string(dof));
    for (int idx : joint_indices_)
        if (idx < 0 || static_cast<size_t>(idx) >= ROBOT_DOF)
            throw std::runtime_error("joint_index must be in [0, " + std::to_string(ROBOT_DOF) + ")");

    q_ = dq_ = torque_ = ref_q_ = ref_dq_ = Eigen::VectorXd::Zero(dof);
    q_target = qdot_target = Eigen::VectorXd::Zero(dof);
    target_pos = leg_init_pos = Eigen::VectorXd::Zero(dof);

    // ---- mode ----
    const std::string mode = config["mode"].as<std::string>("position");
    if (mode == "position") target_mode = POSITION;
    else if (mode == "velocity") target_mode = VELOCITY;
    else if (mode == "torque") throw std::runtime_error("mode: torque is not implemented yet");
    else throw std::runtime_error("mode must be position / velocity / torque (got '" + mode + "')");

    // ---- homing ---- (자세는 command() HOMING 에서 직접 설정)
    homing_max_distance = config["homing"]["max_distance"].as<double>(0.5);

    // ---- torque mode (PC-side PD) ----
    const YAML::Node torque = config["torque"];
    kp = jointParam(torque["kp"], dof, 0.0, "torque.kp");
    kd = jointParam(torque["kd"], dof, 0.0, "torque.kd");
    tau_max = jointParam(torque["tau_max"], dof, 1.0, "torque.tau_max");

    // ---- safety ----
    max_tracking_error = config["safety"]["max_tracking_error"].as<double>(0.5);

    commands_.q_target.assign(dof, 0.0);
    commands_.qd_target.assign(dof, 0.0);
    commands_.torque_target.assign(dof, 0.0);

    traj.init(control_period, dof);
}

Eigen::VectorXd Controller::SelectJoints(const Eigen::VectorXd& pose) const
{
    Eigen::VectorXd v(dof);
    for (size_t i = 0; i < dof; ++i) v(i) = pose(joint_indices_[i]);
    return v;
}

void Controller::sensor_callback(const std::vector<double>& q, const std::vector<double>& dq)
{
    q_ = Eigen::Map<const Eigen::VectorXd>(q.data(), dof);
    dq_ = Eigen::Map<const Eigen::VectorXd>(dq.data(), dof);
    sensor_received = true;
}

void Controller::command(bool flag)
{
    if (!flag || !sensor_received) return;

    control_time += control_period;

    switch (controlmode)
    {
        case INIT:
        {
            if (!Recieved_Mode[INIT])
            {
                std::cout<<"[ROBOT MODE FLAG] : "<<"INIT MODE"<<std::endl;
                std::fill_n(Recieved_Mode, FINISH + 1, false);
                Recieved_Mode[INIT] = true;
            }

            // 첫 feedback 위치를 초기 위치로 저장한다.
            leg_init_pos = q_;

            ref_q_ = q_;
            ref_dq_.setZero();
            q_target = q_;
            qdot_target.setZero();
            controlmode = HOMING;
            // 같은 주기에 HOMING 첫 설정(목표 자세, 거리 검사)까지 끝낸다. 검사 전에 enable 되지 않도록.
            [[fallthrough]];
        }

        case HOMING:
        {
            if (!Recieved_Mode[HOMING]) {
                std::cout<<"[ROBOT MODE FLAG] : "<<"HOMING MODE"<<std::endl;

                // 1단계 자세 (ROBOT_DOF 순서, rad, 절대 관절각)
                Eigen::VectorXd pose(ROBOT_DOF);
                pose << 0.7, 0, -2.58, 0,   0, 0, 2.58, 0,   0, 0, 0, -1.94, 0.785, -0.3,   0, 0, 0, 1.94, -0.785, -0.3;
                target_pos = SelectJoints(pose);
                period = 2.0;
                homing_count = 0;
                traj.ResetTraj();

                std::fill_n(Recieved_Mode, FINISH + 1, false);
                Recieved_Mode[HOMING] = true;

                const double distance = (target_pos - leg_init_pos).cwiseAbs().maxCoeff();
                if (distance > homing_max_distance) {
                    std::cerr << "[SAFETY] homing distance " << distance << " rad > max_distance "
                              << homing_max_distance << " rad" << std::endl;
                    finish("homing target too far");
                    break;
                }
            }

            traj.SetSinusoidalTrajectory(target_pos, leg_init_pos, period);
            ref_q_ = traj.GetRefVar();
            ref_dq_ = traj.GetRefVarDot();

            // 1단계가 끝나면 2단계 자세로. 2단계 자세는 계속 유지한다.
            if (traj.isEnd && homing_count < 1) {
                homing_count++;
                std::cout << "[HOMING] stage 2" << std::endl;

                Eigen::VectorXd pose(ROBOT_DOF);
                pose << 0., -1.043, -1.053, 0,   0, 1.043, 1.053, 0,   1.033, -0.398, 0.466, -0.701, 0.052, -0.061,   -1.033, 0.398, -0.466, 0.701, -0.052, -0.061;
                leg_init_pos = target_pos;
                target_pos = SelectJoints(pose);
                period = 2.0;
                traj.ResetTraj();
            }

            q_target = ref_q_;
            qdot_target = ref_dq_;
            break;
        }

        case FINISH:
            return;
    }

    if (!IsCommandValid()) return;

    // 추종 오차 감시: 너무 벌어지면 정지
    const double error = (ref_q_ - q_).cwiseAbs().maxCoeff();
    if (error > max_tracking_error) {
        std::cerr << "[SAFETY] tracking error " << error << " rad > " << max_tracking_error << " rad" << std::endl;
        finish("tracking error");
        return;
    }

    // torque 관절: PC-side PD
    torque_ = kp.cwiseProduct(ref_q_ - q_) + kd.cwiseProduct(ref_dq_ - dq_);
    torque_ = torque_.cwiseMax(-tau_max).cwiseMin(tau_max);
}

const JointCommands& Controller::send_commands_to_robot()
{
    // INIT / FINISH : 현재 위치 유지, 속도/토크 0 (드라이브는 disable)
    if (!IsCommandValid()) {
        for (size_t i = 0; i < dof; ++i) {
            commands_.q_target[i] = q_(i);
            commands_.qd_target[i] = 0.0;
            commands_.torque_target[i] = 0.0;
        }
        return commands_;
    }

    if (target_mode == POSITION) {
        for (size_t i = 0; i < dof; ++i) {
            commands_.q_target[i] = q_target(i);
            commands_.qd_target[i] = 0.0;
            commands_.torque_target[i] = 0.0;
        }
    }
    else if (target_mode == VELOCITY) {
        for (size_t i = 0; i < dof; ++i) {
            commands_.q_target[i] = q_(i);
            commands_.qd_target[i] = qdot_target(i);
            commands_.torque_target[i] = 0.0;
        }
    }
    else if (target_mode == TORQUE) {
        // TODO: 토크 명령
    }
    return commands_;
}

void Controller::finish(const char* reason)
{
    if (controlmode == FINISH) return;
    std::cout<<"[ROBOT MODE FLAG] : "<<"FINISH MODE (" << reason << ")"<<std::endl;
    controlmode = FINISH;
    torque_.setZero();
}

void Controller::print_status()
{
    static const char* names[] = {"INIT", "HOMING", "FINISH"};
    std::cout << "[CTRL] " << names[controlmode];
    for (size_t i = 0; i < dof; ++i) {
        std::cout << " | j" << i << (command_types_[i] == CMD_TORQUE ? "(tq)" : "(ref)")
                  << " q=" << q_(i) << " ref=" << ref_q_(i)
                  << " err=" << ref_q_(i) - q_(i);
        if (command_types_[i] == CMD_TORQUE) std::cout << " tau=" << torque_(i);
    }
    std::cout << std::endl;
}
