#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <mutex>

//////////////////////////////////////////////////////////////////
// Configuration
//////////////////////////////////////////////////////////////////

constexpr size_t MAX_DOF = 26; // wheel leg 8 + leg 12 + arm 7(gripper포함) = 27

struct MotorSharedState
{
    // ---- Measured ----
    // std::string name{""};
    std::string name;

    double q{0.0};
    double qd{0.0};
    double torque{0.0};

    // ---- Target ----
    double q_target{0.0};    // IMPEDANCE CONTROL 관절 위치제어
    double qd_target{0.0};  // IMPEDANCE CONTROL시에는 관절속도 명령 없음
    double torque_target{0.0}; // IMPEDANCE CONTROL 관절 토크제어

    // ---- Gains ----
    double p_gain{0.0}; // IMPEDANCE CONTROL 관절 P gain 부여
    double d_gain{0.0}; // IMPEDANCE CONTROL 관절 D gain 부여


};

// ------------- double buffer ----------- commands, feedback분리
struct FeedbackFrame {
    uint64_t sequence;
    size_t dof;

    std::array<double, MAX_DOF> q;
    std::array<double, MAX_DOF> qd;
    std::array<double, MAX_DOF> torque;
};

struct CommandFrame {
    uint64_t sequence;
    size_t dof;

    std::array<double, MAX_DOF> q_target;
    std::array<double, MAX_DOF> qd_target;
    std::array<double, MAX_DOF> torque_target;
    std::array<double, MAX_DOF> p_gain;
    std::array<double, MAX_DOF> d_gain;
};
// -------------------------------------------------------------

//////////////////////////////////////////////////////////////////
// 🔹 Shared Memory
//////////////////////////////////////////////////////////////////

struct SharedMemory
{

    //// flag
    

    //////////////////////////////////////////////////////////////
    //  Motor State (Real-Time Thread ↔ Control Thread)
    //////////////////////////////////////////////////////////////
    std::atomic<size_t> motor_count{0};
    std::array<MotorSharedState, MAX_DOF> motors;

    // RT loop가 한 싸이클 끝났는지 표시
    std::atomic<bool> motor_data_ready{false};

    // flag variable
    std::atomic<bool> command_ready{false};
    
    //////////////////////////////////////////////////////////////
    // IMU
    //////////////////////////////////////////////////////////////

    std::array<double, 4> quat{};   // w, x, y, z
    std::array<double, 3> gyro{};   // rad/s
    std::atomic<bool> imu_ready{false};

    //////////////////////////////////////////////////////////////
    // Task / Joystick
    //////////////////////////////////////////////////////////////

    std::atomic<bool> RB_pressed{false};
    std::atomic<bool> LB_pressed{false};
    std::atomic<int> task_mode{0};  // 0: INIT, 1:HOMING, 2:TRAJ, 3:DAMPING

    std::array<double, 3> lin_vel_target{};
    std::array<double, 1> ang_vel_target{};

    //////////////////////////////////////////////////////////////
    // Camera (Non-RT)
    //////////////////////////////////////////////////////////////

    static constexpr size_t CAMERA_WIDTH  = 800;
    static constexpr size_t CAMERA_HEIGHT = 600;
    static constexpr size_t CAMERA_CHANNELS = 3;

    static constexpr size_t CAMERA_FRAME_SIZE =
        CAMERA_WIDTH * CAMERA_HEIGHT * CAMERA_CHANNELS;

    std::array<uint8_t, CAMERA_FRAME_SIZE> camera_frame{};
    std::atomic<bool> new_frame_available{false};
    std::mutex camera_mtx;

    //////////////////////////////////////////////////////////////
    // Constructor
    //////////////////////////////////////////////////////////////

    SharedMemory()
    {
        task_mode.store(0, std::memory_order_relaxed);
    }
};
