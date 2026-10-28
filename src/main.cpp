// transped_mujoco 형식의 Controller로 실제 EtherCAT 모터를 제어하는 실행기.
//
//   MuJoCo 버전                     이 프로젝트
//   Daemon (MuJoCo <-> SHM)    ->   GVdrive (EtherCAT 1 kHz RT thread <-> SHM)
//   sensor_thread / JointControlThread -> ControlLoop (아래, control_frequency)
//
// 관절별 명령 프로토콜은 motors.yaml 의 control_mode 로 정해진다.
//   impedance / position / velocity : Controller 궤적(q_ref, qd_ref)을 그대로 전송, PD는 드라이브
//   torque                          : Controller가 kp/kd(controller.yaml)로 계산한 토크 전송

#include "GVdrive.hpp"
#include "controller.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {
std::atomic<bool> gRun{true};
void onSignal(int) { gRun.store(false); }

CommandType toCommandType(GVTerrain::robot::ControlMode mode)
{
    return mode == GVTerrain::robot::TORQUE ? CMD_TORQUE : CMD_REFERENCE;
}

const char* modeName(GVTerrain::robot::ControlMode mode)
{
    switch (mode) {
        case GVTerrain::robot::POSITION:  return "position";
        case GVTerrain::robot::VELOCITY:  return "velocity";
        case GVTerrain::robot::TORQUE:    return "torque";
        case GVTerrain::robot::IMPEDANCE: return "impedance";
    }
    return "unknown";
}

// controller.yaml mode 로 보낼 명령을 해당 모터 드라이브 모드가 받을 수 있는지
bool isCompatible(TargetMode mode, GVTerrain::robot::ControlMode motor_mode)
{
    switch (mode) {
        case POSITION: return motor_mode == GVTerrain::robot::POSITION || motor_mode == GVTerrain::robot::IMPEDANCE;
        case VELOCITY: return motor_mode == GVTerrain::robot::VELOCITY;
        case TORQUE:   return motor_mode == GVTerrain::robot::TORQUE;
    }
    return false;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cerr << "Usage: EtherCATController <ethercat_iface> <motors.yaml> <controller.yaml>\n";
        return 2;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    try {
        const std::string interface = argv[1];
        const YAML::Node config = YAML::LoadFile(argv[3]);

        // SharedMemory는 camera buffer(약 1.4 MB)를 포함하므로 stack에 두지 않는다.
        auto shm = std::make_unique<SharedMemory>();
        GVTerrain::robot::GVdrive drive(interface, argv[2], shm.get());

        // joint_index (로봇 전체 관절 순서에서의 위치) 는 GVdrive 가 쓰지 않으므로 motors.yaml 에서 직접 읽는다.
        // GVdrive::motors 는 motors.yaml 에 적힌 순서 그대로다. 없으면 등록 순서를 쓴다.
        const YAML::Node motors_node = YAML::LoadFile(argv[2])["motors"];
        if (motors_node.size() != drive.motors.size())
            throw std::runtime_error("motors.yaml motor count mismatch");

        const size_t dof = drive.motors.size();
        std::vector<CommandType> command_types;
        std::vector<int> joint_indices;
        // offset: 관절 0점에서 모터가 보고하는 위치 [rad]. 관절 각도 = 모터 위치 - offset
        std::vector<double> offsets;
        for (size_t i = 0; i < dof; ++i) {
            const auto& motor = drive.motors[i];
            const YAML::Node joint_index = motors_node[i]["joint_index"];
            command_types.push_back(toCommandType(motor.control_mode));
            joint_indices.push_back(joint_index ? joint_index.as<int>() : static_cast<int>(i));
            offsets.push_back(motors_node[i]["offset"].as<double>(0.0));
            std::cout << "[JOINT] " << motor.name << " joint=" << joint_indices.back() << " slave=" << motor.slaveIndex
                      << " offset=" << offsets.back()
                      << " control_mode=" << modeName(motor.control_mode)
                      << (command_types.back() == CMD_TORQUE ? " -> PC PD torque" : " -> drive reference")
                      << std::endl;
        }

        const double frequency = config["control_frequency"].as<double>(500.0);
        if (frequency <= 0.0 || frequency > 1000.0)
            throw std::runtime_error("control_frequency must be in (0, 1000] Hz");
        Controller controller(frequency, config, command_types, joint_indices);

        // controller.yaml mode 와 motors.yaml control_mode 가 맞지 않으면 시작하지 않는다.
        for (const auto& motor : drive.motors) {
            if (!isCompatible(controller.GetTargetMode(), motor.control_mode))
                throw std::runtime_error("motor '" + motor.name + "' control_mode=" + modeName(motor.control_mode)
                                         + " does not match controller.yaml mode");
        }

        if (!drive.scan() || !drive.con()) {
            std::cerr << "[Motor] GVdrive scan/con failed" << std::endl;
            return 1;
        }
        std::cout << "[Motor] started, control " << frequency << " Hz" << std::endl;

        const auto period = std::chrono::nanoseconds(static_cast<long long>(1e9 / frequency));
        auto next = std::chrono::steady_clock::now();
        auto lastStatus = next;
        std::vector<double> q(dof), qd(dof);
        bool first_feedback = true;

        while (gRun.load() && drive.runEtherCatLoop.load()) {
            next += period;

            const bool start_flag = shm->motor_data_ready.load(std::memory_order_acquire);
            if (start_flag) {
                for (size_t i = 0; i < dof; ++i) {
                    q[i] = shm->motors[i].q - offsets[i];
                    qd[i] = shm->motors[i].qd;
                }
                // offset 맞출 때 쓰는 값: 관절을 0점 자세에 두고 실행했을 때의 motor 값이 offset
                if (first_feedback) {
                    first_feedback = false;
                    for (size_t i = 0; i < dof; ++i)
                        std::cout << "[START] " << drive.motors[i].name << " motor=" << shm->motors[i].q
                                  << " joint=" << q[i] << " rad" << std::endl;
                }
                controller.sensor_callback(q, qd);
            }

            controller.command(start_flag);
            const JointCommands& cmd = controller.send_commands_to_robot();

            for (size_t i = 0; i < dof; ++i) {
                shm->motors[i].q_target = cmd.q_target[i] + offsets[i];
                shm->motors[i].qd_target = cmd.qd_target[i];
                shm->motors[i].torque_target = cmd.torque_target[i];
            }
            shm->command_ready.store(controller.IsCommandValid(), std::memory_order_release);

            if (controller.GetControlMode() == FINISH) break;

            const auto now = std::chrono::steady_clock::now();
            if (start_flag && now - lastStatus >= std::chrono::seconds(1)) {
                lastStatus = now;
                controller.print_status();
            }
            std::this_thread::sleep_until(next);
        }

        controller.finish(gRun.load() ? "controller stopped" : "signal");
        shm->command_ready.store(false, std::memory_order_release);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        drive.runEtherCatLoop.store(false);
        pthread_join(drive.etherCatThread, nullptr);
        drive.end_ethercat();
        std::cout << "[Motor] stopped" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "EtherCATController fatal: " << e.what() << std::endl;
        return 1;
    }
}
