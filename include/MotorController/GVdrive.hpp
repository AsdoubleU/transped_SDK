// Copyright (c) 2025 Raion Robotics Inc.
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#ifndef GVDrive_HPP_
#define GVDrive_HPP_

#include <mutex>
#include <string>
#include <atomic>
#include <thread>
#include <cmath>
#include "soem/soem.h"
#include <iostream>
#include <map>
#include <fstream>
#include <vector>
#include <array>
#include <sys/mman.h>
#include <time.h>
#include <errno.h>
#include "yaml-cpp/yaml.h"

// import shared memory
#include "SharedMemory.hpp"

// joystick 
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>

// snapshot thread
#include <deque> 

namespace GVTerrain
{

namespace robot
{

struct Convert
{
  // Common motor paremeters
  static constexpr double torqueScale = 1000.0;

  // Medium size motor parameters (중형)
  static constexpr double resolution_M = 65536; // 엔코더 카운트 최대수
  static constexpr double ratedCurrent_M = 10.5;      
  static constexpr double torqueConst_M = 0.246;     
  static constexpr double ratedTorque_M = ratedCurrent_M * torqueConst_M;   
  static constexpr double gearRatio_M = 10; // 중형 기어비 한번더 점검필요있음
  static constexpr double unitToDeg_M = 360.0 / resolution_M;
  static constexpr double unitToRad_M = 2 * M_PI / resolution_M;
  static constexpr double unitToJointPos_M = unitToRad_M / gearRatio_M;
  static constexpr double unitToMotorPos_M = 1 / unitToJointPos_M;
  static constexpr double unitToJointVel_M = unitToJointPos_M;
  static constexpr double unitToMotorVel_M = 1 / unitToJointVel_M;
  static constexpr double unitToJointToq_M = ratedTorque_M * gearRatio_M / torqueScale;
  static constexpr double unitToMotorToq_M = 1 / unitToJointToq_M;
  
  // Medium High redution motor parameters (중형고감속)
  static constexpr double resolution_HM = 65536; // 엔코더 카운트 최대수
  static constexpr double ratedCurrent_HM = 10.5;     
  static constexpr double torqueConst_HM = 0.246;     
  static constexpr double ratedTorque_HM = ratedCurrent_HM * torqueConst_HM;  
  static constexpr double gearRatio_HM = 22; // 중형고감속 기어비 한번더 점검필요있음
  static constexpr double unitToDeg_HM = 360.0 / resolution_HM;
  static constexpr double unitToRad_HM = 2 * M_PI / resolution_HM;
  static constexpr double unitToJointPos_HM = unitToRad_HM / gearRatio_HM;
  static constexpr double unitToMotorPos_HM = 1 / unitToJointPos_HM;
  static constexpr double unitToJointVel_HM = unitToJointPos_HM;
  static constexpr double unitToMotorVel_HM = 1 / unitToJointVel_HM;
  static constexpr double unitToJointToq_HM = ratedTorque_HM * gearRatio_HM / torqueScale;
  static constexpr double unitToMotorToq_HM = 1 / unitToJointToq_HM;
  
  // Small size motor parameters (소형)
  static constexpr double resolution_S = 65536;  // 엔코더 카운트 최대수
  static constexpr double ratedCurrent_S = 5.1;      
  static constexpr double torqueConst_S = 0.104;       
  static constexpr double ratedTorque_S = ratedCurrent_S * torqueConst_S; 
  static constexpr double gearRatio_S = 7.33; // 소형 기어비 한번더 점검필요있음
  static constexpr double unitToDeg_S = 360.0 / resolution_S;
  static constexpr double unitToRad_S = 2 * M_PI / resolution_S;
  static constexpr double unitToJointPos_S = unitToRad_S / gearRatio_S;
  static constexpr double unitToMotorPos_S = 1 / unitToJointPos_S;
  static constexpr double unitToJointVel_S = unitToJointPos_S;
  static constexpr double unitToMotorVel_S = 1 / unitToJointVel_S;
  static constexpr double unitToJointToq_S = ratedTorque_S * gearRatio_S / torqueScale;
  static constexpr double unitToMotorToq_S = 1 / unitToJointToq_S;
 
};

struct __attribute__ ((packed)) RxPDO
{
  int32_t TargetPosition;
  int32_t TargetVelocity;
  int16_t TargetTorque;
  uint8_t ControlWord;
  uint8_t ModesofOperation;
  uint16_t ImpedancePGain;
  uint16_t ImpedanceDGain;
};

struct __attribute__ ((packed)) TxPDO
{
  int32_t PositionActualValue;
  int16_t TorqueActualValue;
  int32_t VelocityActualSensor;
  uint8_t Statusword;
  uint8_t Temperature;
  uint8_t ErrorCode;
};

enum ControlMode
{
  POSITION = 0,  //ETHERCAT CSP 모드
  VELOCITY = 1, // ETHERCAT CSV 모드
  TORQUE = 2, // ETHERCAT CST 모드
  IMPEDANCE = 3, // IMPEDANCE CONTROL 모드  ==> K_p * (TargetPosition - unitToJointPos * PositionActualValue )- K_d * unitToJointVel * VelocityActualValue + TargetTorque

};

//
enum MotorType
{
  M,
  HM,
  S
};

struct Motor // Motor struct
{
  std::string name;
  int slaveIndex;
  ControlMode control_mode;
  double initialposition = 0.0;
  double targetTorque = 0.0;
  double targetPosition = 0.0;
  double targetVelocity = 0.0;
  double Position = 0.0;
  double Torque = 0.0;
  double Velocity = 0.0;
  uint8_t status = 0;
  uint8_t temperature = 0;
  uint8_t errorCode = 0;
  uint16_t ImpedancePGain = 0;
  uint16_t ImpedanceDGain = 0;

  // encoder
  double pos_scale = 1.0;
  double vel_scale = 1.0;
  double torque_scale = 1.0;

  // command
  double inv_pos_scale = 1.0;
  double inv_vel_scale = 1.0;
  double inv_torque_scale = 1.0;

  MotorType type;
};

enum OPMODE
{
  PWM_DUTY_ZERO = 0,
  IMPEDANCE_CONTROL = 7, // Jointspace PD controller 
  CYCLIC_SYNC_POSITION = 8, // position control CSP
  CYCLIC_SYNC_VELOCITY = 9, // velocity control CSV
  CYCLIC_SYNC_TORQUE = 10,  // torque control CST
  PWM_HALF_DUTY = 11,
  OPEN_LOOP_VOLTAGE_TEST = 12,
  ZERO_CURRENT_CONTROL_TEST = 13,
  OPEN_LOOP_CURRENT_TEST = 14,
  COMMUTATION = 15,
};

enum CiA402CONTROLWORD
{
  DISABLE_VOLTAGE = 0,
  ENABLE_OPERATION = 15,
  FAULT_CLEAR = 128
};

enum CiA402StatusWord : int
{
  NOT_READY_TO_SWITCH_ON = 0,
  READY_TO_SWITCH_ON = 33,
  SWITCHED_ON = 35,
  OPERATION_ENABLED = 39
};

class GVdrive
{
public:
  explicit GVdrive(const std::string& interface, const std::string& config_path, SharedMemory* shared_ptr);
  
  ~GVdrive() { } // TODO : need to add specific funtion ~GVdrive();
  // TODO : Define Destructor

  bool scan();
  bool con();
  // I/O
  void setTargetPosition(const std::string& motor_name, double position);
  void setTargetTorque(const std::string& motor_name, double torque);
  void setTargetVelocity(const std::string& motor_name, double velocity);
  double getPosition(const std::string& motor_name);
  double getVelocity(const std::string& motor_name);
  double getTorque(const std::string& motor_name);

  void* loop(double frequency);  // TODO: need to check change void* loop -> void loop
  bool startEthercat();
  bool end_ethercat();



  static void* loopFunc(void* arg);

  pthread_t etherCatThread; 
  std::mutex driveMutex;
  std::string selectedInterface;
  

  /// counter
  uint8_t loop_cnt = 0;

  /// ethercat
  int slaveCount = 0;
  char IOmap_[4096];
  bool ready = false;
  std::atomic_bool runEtherCatLoop = true;
  
  std::vector<Motor> motors;
  std::map<std::string, int> motor_map_;

  robot::OPMODE ModesofOperation;
  robot::CiA402CONTROLWORD controlWord = robot::CiA402CONTROLWORD::DISABLE_VOLTAGE;
  uint32_t clock_count=0;

private:
    SharedMemory* shared_memory_;

    // RT inline feedback update
    inline void updateMotorFeedback(
        Motor& motor,
        robot::TxPDO* tx);

    uint8_t start_slave_index = 1; // 기본값 1

protected:
  double loopFrequency=1000.0;
  double initialposition=0.0;
  void control();
  bool load_config(const std::string& config_path);
};

}
}
#endif
