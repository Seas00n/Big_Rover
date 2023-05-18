#include "spine/dynamixel_motor.h"
#include "spine/socketcan.h"
#include "spine/minicheetah_motor.h"
#include <iostream>
#include <cmath>

#include "assert.h"

#if defined(_WIN32)
#include <windows.h>
#endif

#include <atomic>
#include <ctime>
#include <chrono>
#include <csignal>
#include <math.h>

using namespace std::chrono;


bool over_current = false;

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

// 接收电机返回读数
void rxThread(SocketCan *sc) {
    sc->clearRxCallback();
    float P_MIN = -12.5f;
    float P_MAX = 12.5f;
    float V_MIN = -30.0f;
    float V_MAX = 30.0f;
    float I_MAX = 18.0f;
    struct can_frame frame {};
    while (true)
    {
      *sc >> frame;
      int id = frame.data[0];
      if (id == 1 || id == 2 || id == 3 || id == 4)
      {
        int p_int = (frame.data[1]<<8)|frame.data[2]; 
        int v_int = (frame.data[3]<<4)|(frame.data[4]>>4);
        int i_int = ((frame.data[4]&0xF)<<8)|frame.data[5];
        int T_int = frame.data[6];
        float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
        float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
        float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
        float T_f = T_int;
        float T = T_f - 40;
        printf("id = %d\tp = %f\tv = %f\ti = %fT = %f\n", id, p, v, i, T);

        // 过流保护
        if(i > 10 || i < -10)
        {
          over_current = true;
          std::cout << "rx exit" << std::endl;
          return;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// 持续发指令给电机使电机返回读数
void txThread(AK10_9Motor *motor, double vel1, double vel2) {
  while (true)
  {
    if (over_current)
    {
      for (int i = 0; i < 4; i++)
      {
        double velocity = 0.0;
        motor[i].setQdDes(velocity); // target velocity
        motor[i].setKpKd(0, 4); // Kd for torque in velocity mode
        motor[i].control();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      std::cout << "tx exit" << std::endl;
      return;
    }
    
    for (int i = 0; i < 4; i++)
    {
      double velocity = 0.0;
      if (i % 2 == 0)
      {
        velocity = vel1 * (int)pow(-1, i); // at least 0.2
      }
      else
      {
        velocity = vel2 * (int)pow(-1, i);
      }
      // aMotor[i].setQDes(10 * (int)pow(-1, i)); // target location
      motor[i].setQdDes(velocity); // target velocity
      motor[i].setKpKd(0, 4); // Kd for torque in velocity mode
      motor[i].control();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

int main() {
  // Initialize

  SocketCan can("can0", 1e6);

  // Steering
  DynamixelMotor dMotor[4] = {
    DynamixelMotor(1, "/dev/ttyUSB0"), DynamixelMotor(2, "/dev/ttyUSB0"),
    DynamixelMotor(3, "/dev/ttyUSB0"), DynamixelMotor(4, "/dev/ttyUSB0")};
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Motor
  // AK10_9Motor motor0, motor1, motor2, motor3;
  AK10_9Motor aMotor[4];
  for (int i = 0; i < 4; i++)
  {
    aMotor[i].attach(i + 1, &can);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::thread canRx(rxThread, &can);
  canRx.detach();

  // Initialize End


  // Control
  // std::cout << "Press any key to continue";
  // std::cin.get();

  // Steering
  for (auto &dm : dMotor)
  {
    dm.setQDes(0); // target angle
    dm.control();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Motor
  double vel1 = 0.4;
  double vel2 = 0.4;
  for (int i = 0; i < 4; i++)
  {
    double velocity = 0.0;
    if (i % 2 == 0)
    {
      velocity = vel1 * (int)pow(-1, i); // at least 0.2
    }
    else
    {
      velocity = vel2 * (int)pow(-1, i);
    }
    // aMotor[i].setQDes(10 * (int)pow(-1, i)); // target location
    aMotor[i].setQdDes(velocity); // target velocity
    aMotor[i].setKpKd(0, 4); // Kd for torque in velocity mode
    aMotor[i].control();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::thread canTx(txThread, aMotor, vel1, vel2);
  canTx.detach();

  // Control End

  // char perms[]="rwxrwxr-x"; // 509
  // int bits = 0;
  // for(int i=0; i<9; i++){
  //   if (perms[i] != '-') {
  //     bits |= 1<<(8-i);
  //   }
  // }
  // printf("%d\n", bits);
  
  // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  return 0;
}
