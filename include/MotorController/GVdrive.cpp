
#include "GVdrive.hpp"
#include <sstream>
#include <pthread.h>
#include <sched.h>

#define CPU_CORE 3

namespace GVTerrain::robot { // Convert 

  inline void GVdrive::updateMotorFeedback(
    Motor& motor,
    robot::TxPDO* tx)

  {
      motor.Position =
          tx->PositionActualValue * motor.pos_scale;

      motor.Velocity =
          tx->VelocityActualSensor * motor.vel_scale;

      motor.Torque =
          tx->TorqueActualValue * motor.torque_scale;
  }

  

  GVdrive::GVdrive(const std::string& interface, const std::string& config_path, SharedMemory* shared_ptr)
  : selectedInterface(interface),
    shared_memory_(shared_ptr)   
  { // load config

    runEtherCatLoop = true;

    if (!load_config(config_path)) {
      throw std::runtime_error("Failed to load or motor configuration file");
    }


    if (shared_memory_)
    {
        shared_memory_->motor_count.store(
            motors.size(),  // Motor struct vector size.
            std::memory_order_relaxed);
    }

    if (motors.size() > MAX_DOF)
    {
    throw std::runtime_error(
        "Motor count exceeds SharedMemory MAX_DOF");
    }


    if (shared_memory_) {
        for (size_t i = 0; i < motors.size(); ++i) {
            shared_memory_->motors[i].p_gain = static_cast<double>(motors[i].ImpedancePGain);  // motors.yaml에서 로드
            shared_memory_->motors[i].d_gain = static_cast<double>(motors[i].ImpedanceDGain);  // motors.yaml에서 로드

            shared_memory_->motors[i].name = motors[i].name;
        }
        std::cout << "[INIT] Initial Gains (P/D) pushed to SharedMemory from YAML." << std::endl;
    }

  }




  bool GVdrive::load_config(const std::string& config_path)  // load motor info by user enrollmented
  {
      try
      {
          YAML::Node config = YAML::LoadFile(config_path);


          if (config["use_ethercat_hub"] && config["use_ethercat_hub"].as<bool>()) { // if use ethercat hub.
            start_slave_index = 3;  // 허브 MCU가 1,2 차지 -> 3 INDEX시작
            std::cout << "[CONFIG] EtherCAT Hub detected. Slaves start from index 3." << std::endl;
          } else {
              start_slave_index = 1;
              std::cout << "[CONFIG] Direct connection detected. Slaves start from index 1." << std::endl;
          }


          const auto& motors_node = config["motors"];

          if (!motors_node)
          {
              std::cerr << "YAML file doesn't contain 'motors' key" << std::endl;
              return false;
          }


          motors.clear(); // vector
          motor_map_.clear(); // map : string, int

          for (const auto& node : motors_node)
          {
              Motor m; // 구조체  // each motor

              // -------------------------------------------------
              //  Basic Info
              // -------------------------------------------------
              m.name = node["name"].as<std::string>();
              m.slaveIndex = node["slave_index"].as<int>();
              m.ImpedancePGain = node["p_gain"].as<uint16_t>();
              m.ImpedanceDGain = node["d_gain"].as<uint16_t>();


              // -------------------------------------------------
              //  Control Mode (제어 방식만)
              // -------------------------------------------------
              std::string mode_str = node["control_mode"].as<std::string>();

              if (mode_str == "torque")
                  m.control_mode = TORQUE;
              else if (mode_str == "position")
                  m.control_mode = POSITION;
              else if (mode_str == "velocity")
                  m.control_mode = VELOCITY;
              else if (mode_str == "impedance")
                  m.control_mode = IMPEDANCE;
              else
              {
                  std::cerr << "Unknown control_mode: " << mode_str
                            << " → default TORQUE" << std::endl;
                  m.control_mode = TORQUE;
              }


              // -------------------------------------------------
              // Motor Type (하드웨어 타입)
              // -------------------------------------------------
              std::string type_str = node["type"].as<std::string>();

              
              if (type_str == "M")
                  m.type = M;
              else if (type_str == "HM")
                  m.type = HM;
              else if (type_str == "S")
                  m.type = S;
              else
              {
                  std::cerr << "Unknown motor type: " << type_str
                            << " → default M" << std::endl;
                  m.type = M;
              }


              // -------------------------------------------------
              // Scale 설정 (MotorType 기준)
              // -------------------------------------------------
              switch (m.type)
              {
                  case M:
                      m.pos_scale    = Convert::unitToJointPos_M;
                      m.vel_scale    = Convert::unitToJointVel_M;
                      m.torque_scale = Convert::unitToJointToq_M;
                      break;

                  case HM:
                      m.pos_scale    = Convert::unitToJointPos_HM;
                      m.vel_scale    = Convert::unitToJointVel_HM;
                      m.torque_scale = Convert::unitToJointToq_HM;
                      break;

                  case S:
                      m.pos_scale    = Convert::unitToJointPos_S;
                      m.vel_scale    = Convert::unitToJointVel_S;
                      m.torque_scale = Convert::unitToJointToq_S;
                      break;

                  default:
                      m.pos_scale    = 1.0;
                      m.vel_scale    = 1.0;
                      m.torque_scale = 1.0;
                      break;
              }

              // -------------------------------------------------
              // Inverse scale (command 변환용)
              // -------------------------------------------------
              m.inv_pos_scale    = 1.0 / m.pos_scale;
              m.inv_vel_scale    = 1.0 / m.vel_scale;
              m.inv_torque_scale = 1.0 / m.torque_scale;


              // -------------------------------------------------
              // 저장
              // -------------------------------------------------
              motors.push_back(m); // struct vector
              motor_map_[m.name] = motors.size() - 1;  // name , index
          }

          std::cout << "Load Success: "
                    << motors.size()
                    << " motors from "
                    << config_path << std::endl;

          return true;
      }
      catch (const YAML::Exception& e)
      {
          std::cerr << "Error parsing YAML file: "
                    << e.what() << std::endl;
          return false;
      }
  }


  
  bool GVdrive::scan() {
    if (ec_init(selectedInterface.c_str())) {
      slaveCount = ec_config_init(false);
      std::cout<<"initialized ethercat"<<std::endl; 
      
      std::cout << "[SCAN] ec_config_init returned slaveCount = " << slaveCount << std::endl; // 1 


      // PDO MAPPING 이전이라, Obits,ibits는 수정해야할 legacy임

      for (int i = 1; i <= slaveCount; ++i) { // 0 index = Master PC -> 1 index부터 ethercat slave count
        std::cout << "[SCAN] slave " << i
                  << " | name: " << ec_slave[i].name
                  << " | state: " << ec_slave[i].state
                  << " | obits: " << ec_slave[i].Obits
                  << " | ibits: " << ec_slave[i].Ibits
                  << std::endl;}

    } else {
      std::cerr << "ethercat initialization failed"<<std::endl;
      return false;
    }
    return true;
  }

  bool GVdrive::con() {


    if(!startEthercat()){
      std::cerr << "startEtherCat() failed"<<std::endl;
      return false;
    }

    if (pthread_create(&etherCatThread, nullptr, GVdrive::loopFunc, this) != 0) {
      perror("Failed to create EtherCAT thread");
      return false;
    }


    return true;
  }

  bool GVdrive::startEthercat() {
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    ec_config_overlap_map(&IOmap_);

    //------------------------------
    
    // expectedWKC = 
    //     ec_group[0].outputsWKC * 2
    //     + ec_group[0].inputsWKC;

    // std::cout << "outputsWKC: " << ec_group[0].outputsWKC << '\n';
    // std::cout << "inputsWKC : " << ec_group[0].inputsWKC << '\n';
    // std::cout << "expectedWKC: " << expectedWKC << '\n';

    // for (int i = 1; i <= ec_slavecount; ++i) {
    //     std::cout << "slave " << i
    //               << " Obytes=" << ec_slave[i].Obytes
    //               << " Ibytes=" << ec_slave[i].Ibytes
    //               << " Obits=" << ec_slave[i].Obits
    //               << " Ibits=" << ec_slave[i].Ibits
    //               << '\n';
    // }



    //--------------------------------

    ec_configdc();
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_overlap_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);
    //////////// wait for all slaves to reach OP state ////////////////////////////
    uint8_t count = 5; // TODO : change up to 100
    uint16_t timeout = 50000;
    do {
      ec_send_overlap_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_statecheck(0, EC_STATE_OPERATIONAL, timeout);
    
    } while (count-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
    
    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
      std::cout<<"all slaves are operational"<<std::endl;
    }
    else{
      ec_readstate();
      std::cerr << "not all slaves reached operational state" << std::endl;
    }
    std::cout << "ec_slavecount : "<< ec_slavecount << std::endl; // ec_slave[0] is reserved for the master.

        std::cout << "----- YAML motor mapping check -----" << std::endl;
    for (const auto& motor : motors) {
      std::cout << "[YAML] motor=" << motor.name
                << " | slave_index=" << motor.slaveIndex
                << " | ec_slavecount=" << ec_slavecount
                << std::endl;

      if (motor.slaveIndex < 1 || motor.slaveIndex > ec_slavecount) {
        std::cerr << "[FATAL] Invalid slave_index for motor '" << motor.name
                  << "': " << motor.slaveIndex
                  << " (valid range: 1 ~ " << ec_slavecount << ")"
                  << std::endl;
        return false;
      }

      if (ec_slave[motor.slaveIndex].outputs == nullptr) {
        std::cerr << "[FATAL] outputs is null for motor '" << motor.name
                  << "' at slave " << motor.slaveIndex << std::endl;
        return false;
      }

      if (ec_slave[motor.slaveIndex].inputs == nullptr) {
        std::cerr << "[FATAL] inputs is null for motor '" << motor.name
                  << "' at slave " << motor.slaveIndex << std::endl;
        return false;
      }
    }
    std::cout << "-----------------------------------" << std::endl;


    for (uint8_t i = start_slave_index; i <= ec_slavecount; i++) { // ec_slave[0] is reserved for the master.
      auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[i].outputs);
      rx->TargetPosition = 0;
      rx->TargetVelocity = 0;
      rx->TargetTorque = 0;
      rx->ControlWord = robot::CiA402CONTROLWORD::FAULT_CLEAR;
      rx->ModesofOperation = robot::OPMODE::PWM_DUTY_ZERO;
      rx->ImpedancePGain = 0;
      rx->ImpedanceDGain = 0;
    }
    ready = true;
    ///////////////////////// fault clear //////////////////////////////////
    std::map<uint8_t, int> initialErrorCodes;

    // Helper function to get error descriptions
    auto getErrorDescription = [](int errorCode) -> std::string {
      std::stringstream ss;
      ss << "Errors: ";
      if (errorCode & (1 << 0)) { ss << "[Over Current] "; }
      if (errorCode & (1 << 1)) { ss << "[Over Temperature] "; }
      if (errorCode & (1 << 2)) { ss << "[No Encoder Offset] "; }
      return ss.str();
    };
    // Helper function to convert to binary string
    auto toBinaryString = [](int value) -> std::string {
      std::string binary;
      for (int i = 4; i >= 0; i--) {  // Changed to 4 bits for integer
        // TODO : for loop i=3 -> i=4 need to check 
        binary += (value & (1 << i)) ? '1' : '0';
        if (i % 4 == 0 && i != 0) {
          binary += ' ';                           // Add space every 4 bits
        }
      }
      return binary;
    };
    
    // First pass: record initial error codes
    for (uint8_t i = start_slave_index; i <= ec_slavecount; i++) { // ec_slave[0] is reserved for the master.
      auto tx = reinterpret_cast<robot::TxPDO *>(ec_slave[i].inputs);
      if (tx->ErrorCode != 0) {
        initialErrorCodes[i] = tx->ErrorCode;
        std::cerr << std::to_string(i) + ": fault code "
                            + std::to_string((int) tx->ErrorCode)
                            + " (binary: " + toBinaryString(tx->ErrorCode) + ") "
                            + getErrorDescription(tx->ErrorCode) <<std::endl;
      }
    }
    
    // Clear faults loop
    bool noErrorForAllSlaves;
    int faultClearCnt = 100;
    do {
      ec_send_overlap_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      noErrorForAllSlaves = true;
      for (uint8_t i = start_slave_index; i <= ec_slavecount; i++) { // ec_slave[0] is reserved for the master.
        auto tx = reinterpret_cast<robot::TxPDO *>(ec_slave[i].inputs);
        auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[i].outputs);
        rx->TargetPosition = 0;
        rx->TargetVelocity = 0;
        rx->TargetTorque = 0;
        rx->ControlWord = robot::CiA402CONTROLWORD::FAULT_CLEAR;
        rx->ModesofOperation = robot::OPMODE::PWM_DUTY_ZERO;
        rx->ImpedancePGain = 0;
        rx->ImpedanceDGain = 0;
        // If fault was present but now cleared
        if (tx->ErrorCode == 0 && initialErrorCodes.count(i) > 0) {
          std::cerr <<  std::to_string(i) + " (was: "
                              + std::to_string(initialErrorCodes[i]) + ", binary: "
                              + toBinaryString(initialErrorCodes[i]) + ")" << std::endl;
          initialErrorCodes.erase(i);
        }
        if (tx->ErrorCode != 0) {
          noErrorForAllSlaves = false;
        }
    
      }
    } while (faultClearCnt-- && !noErrorForAllSlaves);
  
  
    if (!noErrorForAllSlaves)
    {
      std::cerr << "not all faults cleared for all slaves." <<std::endl;
  
      for (uint8_t i = start_slave_index; i <= ec_slavecount; i++) { // ec_slave[0] is reserved for the master.
        auto tx = reinterpret_cast<robot::TxPDO *>(ec_slave[i].inputs);
        if (tx->ErrorCode != 0) {
          initialErrorCodes[i] = tx->ErrorCode;
          std::cerr << 
                          std::to_string(i) + ": fault code "
                              + std::to_string((int) tx->ErrorCode) + ", (binary: "
                              + toBinaryString(tx->ErrorCode) + ") "
                              + getErrorDescription(tx->ErrorCode) <<std::endl;
        }
      }
    } else {
      std::cout << "all faults cleared for all slaves." <<std::endl;
    }
  
    for (uint8_t i = start_slave_index; i <= ec_slavecount; i++) { // ec_slave[0] is reserved for the master.
      auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[i].outputs);
      rx->TargetPosition = 0;
      rx->TargetVelocity = 0;
      rx->TargetTorque = 0;
      rx->ControlWord = robot::CiA402CONTROLWORD::DISABLE_VOLTAGE;
      rx->ModesofOperation = robot::OPMODE::PWM_DUTY_ZERO;
      rx->ImpedancePGain = 0;
      rx->ImpedanceDGain = 0;
    }

   
    std::cout<< "startEthercat() complete"<<std::endl;


    return true;  
  }

  
  //안전 종료 코드// <- NSCL 
  bool GVdrive::end_ethercat() 
  { for (uint8_t i = start_slave_index; i <= ec_slavecount; i++) // ec_slave[0] is reserved for the master.
    { 
      auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[i].outputs); 
      rx -> TargetPosition = 0; 
      rx -> TargetVelocity = 0; 
      rx -> TargetTorque = 0; 
      rx -> ControlWord = 0x0000; 
      rx -> ModesofOperation = 0; 
      rx -> ImpedancePGain = 0; 
      rx -> ImpedanceDGain = 0; } 
    ec_send_overlap_processdata(); 
    ec_receive_processdata(EC_TIMEOUTRET); 
    ec_slave[0].state = EC_STATE_INIT; ec_writestate(0); 
    ec_close(); 
    std::cout<< "end_Ethercat() complete"<<std::endl; return true;
  //control_input_key//
  }


  void* GVdrive::loop(double frequency) {

    // Calculate the period as duration in seconds
    long period_ns=static_cast<long>(1e9/frequency);

    // Determine the next tick time points
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    
    while (runEtherCatLoop) {
      GVdrive::control();

    
    next.tv_nsec+=period_ns;
    while(next.tv_nsec>=1000000000L){
      next.tv_nsec -=1000000000L;
      next.tv_sec+=1;
    }
    int ret;
    do{ 
      ret=clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }while(ret&&errno==EINTR);
  }

  return nullptr; // TODO : change pointer void function to pointer funtion
  }

  void* GVdrive::loopFunc(void* arg){ // ethercatPthread entry funtion
    GVdrive* self =static_cast<GVdrive*>(arg);

    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(CPU_CORE, &cpu_set);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
      perror("memory lockall failed");
      exit(-1);
    }

    if(pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set) != 0){
      perror("pthread_setaffinity_np failed");
    }

    struct sched_param schedParam;
    schedParam.sched_priority = 99;
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedParam)!=0){
      perror("pthread_setschedparam failed");
    }

    // TODO check return nullptr
    // self->loop(self->loopFrequency);
    // return nullptr;
    return self->loop(self->loopFrequency);
  }


  void GVdrive::control(){

    
    if (shared_memory_)
        
    {
        

        size_t dof = 0; // yaml's motor dof initialize

        if (shared_memory_)
        {
            size_t count =
                shared_memory_->motor_count.load(
                    std::memory_order_relaxed);

            dof = std::min(count, motors.size()); // load_config로부터 motor의 수 할당하여 제어대상 객체의 shared memory 구조체 연결
        }

        // ======================================================
        // SAFETY GATE:
        // main/user loop가 valid q_target을 쓰기 전에는
        // shared_memory의 초기값(보통 0)을 모터 명령으로 사용하지 않는다.
        // ======================================================
        if (!shared_memory_->command_ready.load(std::memory_order_acquire))
        {
            for (auto &motor : motors)
            {
                auto rx = reinterpret_cast<robot::RxPDO *>(
                    ec_slave[motor.slaveIndex].outputs);

                rx->TargetPosition = 0;
                rx->TargetVelocity = 0;
                rx->TargetTorque = 0;
                rx->ControlWord = robot::CiA402CONTROLWORD::DISABLE_VOLTAGE;
                rx->ModesofOperation = robot::OPMODE::PWM_DUTY_ZERO;
                rx->ImpedancePGain = 0;
                rx->ImpedanceDGain = 0;
            }

            ec_send_overlap_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            // feedback은 계속 publish해서 main loop가 현재 q를 받을 수 있게 한다.
            for (size_t i = 0; i < dof; ++i)
            {
                auto &motor = motors[i];
                auto tx = reinterpret_cast<robot::TxPDO *>(
                    ec_slave[motor.slaveIndex].inputs);

                updateMotorFeedback(motor, tx);

                shared_memory_->motors[i].q = motor.Position;
                shared_memory_->motors[i].qd = motor.Velocity;
                shared_memory_->motors[i].torque = motor.Torque;
            }

            shared_memory_->motor_data_ready.store(
                true, std::memory_order_release);

            return;
        }

        // shared_memory command -> motors info 갱신

        for (size_t i = 0; i < dof; ++i)
        {
            motors[i].targetPosition = shared_memory_->motors[i].q_target;  
            motors[i].targetVelocity = shared_memory_->motors[i].qd_target;
            motors[i].targetTorque   = shared_memory_->motors[i].torque_target;
            
            motors[i].ImpedancePGain =
                static_cast<uint16_t>(shared_memory_->motors[i].p_gain);
            motors[i].ImpedanceDGain =
                static_cast<uint16_t>(shared_memory_->motors[i].d_gain);
        }

        
    }

    for(auto &motor:motors){ // motors 벡터 내의 각 구동기의 구조체 info를 통해 ec_slave outpus값 갱신
      auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[motor.slaveIndex].outputs);  // RxPDO 드라이버 명령 구조체
      
      switch (motor.control_mode)
      {
          case TORQUE:
              rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_TORQUE;
              rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
              rx->TargetTorque = motor.targetTorque * motor.inv_torque_scale; // 위 shared memory 입력 넣음
              break;

          case POSITION:
              rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_POSITION;
              rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
              rx->TargetPosition = motor.targetPosition * motor.inv_pos_scale; // 위 shared memory 입력 넣음
              break;

          case VELOCITY:
              rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_VELOCITY;
              rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
              rx->TargetVelocity = motor.targetVelocity * motor.inv_vel_scale; // 위 shared memory 입력 넣음
              break;

          case IMPEDANCE:
              rx->ModesofOperation = robot::OPMODE::IMPEDANCE_CONTROL;
              rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
              rx->TargetPosition = motor.targetPosition * motor.inv_pos_scale; // 위 shared memory 입력 넣음
              rx->TargetTorque = motor.targetTorque * motor.inv_torque_scale; // 위 shared memory 입력 넣음
                              
              rx->ImpedancePGain = motor.ImpedancePGain; // IMPEDANCEONTOL 
              rx->ImpedanceDGain = motor.ImpedanceDGain; // IMPEDANCECONTROL

              break;

          default:
              rx->ModesofOperation = robot::OPMODE::PWM_DUTY_ZERO;
              rx->ControlWord = robot::CiA402CONTROLWORD::DISABLE_VOLTAGE;
              break;
       }

      }

    
      ec_send_overlap_processdata();
      int workCount = ec_receive_processdata(EC_TIMEOUTRET);  // RxPDO명령 갱신후 , WorkCount 수 변동있는지 계속 확인

    
    {   
        

        size_t dof = 0;

        if (shared_memory_)
        {
            size_t count =
                shared_memory_->motor_count.load(
                    std::memory_order_relaxed);

            dof = std::min(count, motors.size());

            
        }



        for (size_t i = 0; i < dof; ++i)
        {
            auto &motor = motors[i];
            auto tx = reinterpret_cast<robot::TxPDO *>(  // 엔코더 센서의 현재 state값을 shared memory에 갱신
                ec_slave[motor.slaveIndex].inputs);

            updateMotorFeedback(motor, tx);

            // ======================================================
            //  Publish to SharedMemory (NO mutex)
            // ======================================================

            // gvdrive.hpp 헤더의 motors 와 shared memory의 motors는 yaml에서 count 순서대로 초기화됨
            if (shared_memory_)
            {
                shared_memory_->motors[i].q =
                    motor.Position;

                shared_memory_->motors[i].qd =
                    motor.Velocity;

                shared_memory_->motors[i].torque =
                    motor.Torque;
            }
        }

        if (shared_memory_)
        {
            shared_memory_->motor_data_ready.store(
                true, std::memory_order_release);
        }
    }

  }
}
