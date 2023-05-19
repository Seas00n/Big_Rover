#include <iostream>

#include "spine/socketcan.h"
#include "spine/dynamixel_motor.h"
#include "spine/minicheetah_motor.h"
#include "rover_control/corner.h"

static const float P_MIN = -12.5f;
static const float P_MAX = 12.5f;
static const float V_MIN = -30.0f;
static const float V_MAX = 30.0f;
static const float I_MAX = 18.0f;


struct wheelMotorTypeDef{
    int id;
    float pos_actual;
    float pos_desired;
    float vel_actual;
    float vel_desired;
};

struct steeringMotorTypeDef{
    float pos_actual;
    float pos_desired;
};

struct RoverTypeDef{
    float rover_v;
    float rover_w; 
    int rover_motion_state;
};


enum rover_state{GO_AHEAD=0,CHANGE_TO_TURN,TURN,CHANGE_TO_GO};

std::vector<wheelMotorTypeDef> wheels;
std::vector<steeringMotorTypeDef> steers; 

void thread_sleep(int ms){
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
int depackMsg(struct can_frame & frame){
    int id = frame.data[0];
    int p_int = (frame.data[1]<<8)|frame.data[2]; 
    int v_int = (frame.data[3]<<4)|(frame.data[4]>>4);
    int i_int = ((frame.data[4]&0xF)<<8)|frame.data[5];
    int T_int = frame.data[6];
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
    float T_f = T_int;
    float T = T_f - 40;
    if(abs(i)>I_MAX){
        return 0;//over current
    }
    if(i==0||i==1||i==2||i==3){
        wheels[i].pos_actual = p;
        wheels[i].vel_actual = v;
    }
    return 1;
}

void UpdateMessage(std::vector<rover_control::corner> corners){
    for(int i=0;i<wheels.size();i++){
        corners[i].wheel_pos_actual = wheels[i].pos_actual;
        corners[i].wheel_pos_desired = wheels[i].pos_desired;
        corners[i].wheel_vel_actual = wheels[i].vel_actual;
        corners[i].wheel_vel_desired = wheels[i].vel_desired;
        corners[i].steer_pos_actual = steers[i].pos_actual;
        corners[i].steer_pos_desired = steers[i].pos_desired;
    }
}