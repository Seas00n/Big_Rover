#include <iostream>
#include <math.h>
#include "spine/socketcan.h"
#include "spine/dynamixel_motor.h"
#include "spine/minicheetah_motor.h"
#include "msg_.hpp"
#include "rover_control/corner.h"
#include "turtlesim/Pose.h"
#include "ros/ros.h"
extern std::vector<wheelMotorTypeDef> wheels;//车轮软件抽象
extern std::vector<steeringMotorTypeDef> steers;//舵机软件抽象
struct RoverTypeDef rover_temp;

bool is_thread_ok = true;

#define PI 3.141592657
#define wheel_to_center_x 10.0
#define wheel_to_center_y 10.0
#define wheel_to_center sqrt(pow(wheel_to_center_x,2)+pow(wheel_to_center_y,2))
#define cal_distance(x,y) sqrt(pow(x,2)+pow(y,2))

void canRxThread(SocketCan *sc){
  sc->clearRxCallback();
  struct can_frame frame
  {
    /* data */
  };
  while(is_thread_ok){
    *sc >> frame;
    if(depackMsg(frame)){

    }else{
      is_thread_ok = false;
      return;
    }
    thread_sleep(5);
  }
}


void cmdTxThread(std::vector<AK10_9Motor> wheel_Motor,
                  std::vector<DynamixelMotor> steering_Motor){
    while(is_thread_ok){
      for(int i=0;i<wheel_Motor.size();i++){
        steering_Motor[i].setQDes(steers[i].pos_desired);
        wheel_Motor[i].setQdDes(wheels[i].vel_desired);
        wheel_Motor[i].setKpKd(0,4);
        steering_Motor[i].control();
        thread_sleep(5);
        wheel_Motor[i].control();
        thread_sleep(5);
      }
    }
}
void BicycleControl(const turtlesim::Pose::ConstPtr& p){
  rover_temp.rover_v = p->linear_velocity;
  rover_temp.rover_w = p->angular_velocity;
  wheels[3].vel_desired = rover_temp.rover_v;
  wheels[4].vel_desired = rover_temp.rover_v;
  double steer_angle = atan2(rover_temp.rover_w*2*wheel_to_center_y/rover_temp.rover_v,1.0);
  double turn_radius = 2*wheel_to_center_y/(tan(steer_angle));
  steers[0].pos_desired = steer_angle;
  steers[1].pos_desired = steer_angle;
  steers[2].pos_desired = 0;
  steers[3].pos_desired = 0;
  wheels[0].vel_desired = rover_temp.rover_v/turn_radius*cal_distance(turn_radius,2*wheel_to_center_y);
  wheels[1].vel_desired = wheels[0].vel_desired;
}

void AckermanControl(const turtlesim::Pose::ConstPtr& p){
  rover_temp.rover_v = p->linear_velocity;
  rover_temp.rover_w = p->angular_velocity;
  wheels[3].vel_desired = rover_temp.rover_v;
  wheels[4].vel_desired = rover_temp.rover_v;
  double steer_angle = atan2(rover_temp.rover_w*2*wheel_to_center_y/rover_temp.rover_v,1.0);
  double turn_radius = 2*wheel_to_center_y/(tan(steer_angle));
  steers[0].pos_desired = atan2(2*wheel_to_center_y,turn_radius+wheel_to_center_x);
  steers[1].pos_desired = atan2(2*wheel_to_center_y,turn_radius-wheel_to_center_x);
  steers[2].pos_desired = 0;
  steers[3].pos_desired = 0;
  wheels[0].vel_desired = rover_temp.rover_v/turn_radius*cal_distance(turn_radius+wheel_to_center_x,2*wheel_to_center_y);
  wheels[1].vel_desired = rover_temp.rover_v/turn_radius*cal_distance(turn_radius-wheel_to_center_x,2*wheel_to_center_y);
}

void FSMControl(const turtlesim::Pose::ConstPtr& p){
  if(rover_temp.rover_motion_state==GO_AHEAD){
    rover_temp.rover_v = p->linear_velocity;
    rover_temp.rover_w = 0;
    for(int i=0;i<4;i++){
      wheels[i].vel_desired = rover_temp.rover_v;
      steers[i].pos_desired = 0;
    }
    if(abs(p->angular_velocity)>0.1){
      rover_temp.rover_motion_state = CHANGE_TO_TURN;
    }
  }else if (rover_temp.rover_motion_state = CHANGE_TO_TURN){
    rover_temp.rover_v = 0;
    rover_temp.rover_w = 0;
    steers[0].pos_desired = 45.0*PI/180.0;
    steers[1].pos_desired = -45.0*PI/180.0;
    steers[2].pos_desired = -45.0*PI/180.0;
    steers[3].pos_desired = 45.0*PI/180.0;
    double error = (steers[0].pos_actual+steers[1].pos_actual+steers[2].pos_actual+steers[3].pos_actual)-
                    (steers[0].pos_actual+steers[1].pos_actual+steers[2].pos_actual+steers[3].pos_actual);
    if(abs(error)<0.01){
      rover_temp.rover_motion_state = TURN;
    }
  }else if(rover_temp.rover_motion_state == TURN){
    rover_temp.rover_v = 0;
    rover_temp.rover_w = p->angular_velocity;
    double wheel_turn = rover_temp.rover_w*wheel_to_center;
    for(int i=0;i<4;i++){
      wheels[i].vel_desired = wheel_turn;
    }
    steers[0].pos_desired = 45.0*PI/180.0;
    steers[1].pos_desired = -45.0*PI/180.0;
    steers[2].pos_desired = -45.0*PI/180.0;
    steers[3].pos_desired = 45.0*PI/180.0;
    if(abs(p->linear_velocity)>0.5){
      rover_temp.rover_motion_state = CHANGE_TO_GO;
    }
  }else if(rover_temp.rover_motion_state = CHANGE_TO_GO){
    rover_temp.rover_v = 0;
    rover_temp.rover_w = 0;
    steers[0].pos_desired = 0;
    steers[1].pos_desired = 0;
    steers[2].pos_desired = 0;
    steers[3].pos_desired = 0;
    double error = (steers[0].pos_actual+steers[1].pos_actual+steers[2].pos_actual+steers[3].pos_actual)-
                    (steers[0].pos_actual+steers[1].pos_actual+steers[2].pos_actual+steers[3].pos_actual);
    if(abs(error)<0.01){
      rover_temp.rover_motion_state = GO_AHEAD;
    }
  }
}


int main(int argc, char *argv[]){
  SocketCan can("can0",1e6);
  std::vector<DynamixelMotor> steering_Motor;//舵机硬件抽象
  steering_Motor.push_back(DynamixelMotor(1,"/dev/ttyUSB0"));
  steering_Motor.push_back(DynamixelMotor(2,"/dev/ttyUSB0"));
  steering_Motor.push_back(DynamixelMotor(3,"/dev/ttyUSB0"));
  steering_Motor.push_back(DynamixelMotor(4,"/dev/ttyUSB0"));
  thread_sleep(5);

  std::vector<AK10_9Motor> wheel_Motor;//车轮硬件抽象
  wheel_Motor.push_back(AK10_9Motor());
  wheel_Motor.push_back(AK10_9Motor());
  wheel_Motor.push_back(AK10_9Motor());
  wheel_Motor.push_back(AK10_9Motor());
  wheel_Motor[0].attach(1,&can);thread_sleep(5);
  wheel_Motor[1].attach(2,&can);thread_sleep(5);
  wheel_Motor[2].attach(3,&can);thread_sleep(5);
  wheel_Motor[3].attach(4,&can);thread_sleep(5);

  setlocale(LC_ALL, "");
  ros::init(argc, argv, "control");
  ros::NodeHandle nh;  
  ros::Rate loop_rate(500);
  ROS_INFO_STREAM("节点初始化完成");
  //分布电机数据便于存储
  std::vector<rover_control::corner> corners;
  corners.push_back(rover_control::corner());
  corners.push_back(rover_control::corner());
  corners.push_back(rover_control::corner());
  corners.push_back(rover_control::corner());

  ros::Publisher pub1 = nh.advertise<rover_control::corner>("corner1",100);
  ros::Publisher pub2 = nh.advertise<rover_control::corner>("corner2",100);
  ros::Publisher pub3 = nh.advertise<rover_control::corner>("corner3",100);
  ros::Publisher pub4 = nh.advertise<rover_control::corner>("corner4",100);
  //订阅无人车状态便于控制
  ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose",1000,BicycleControl);
  //初始化所有电机
  for(int i=0;i<wheels.size();i++){
    wheels[i].pos_desired = 0;
    wheels[i].vel_desired = 0;
    steers[i].pos_desired = 0;
  }

  std::thread canRx(canRxThread, &can);
  canRx.detach();
  std::thread cmdTx(cmdTxThread, wheel_Motor,steering_Motor);
  cmdTx.detach();
  while(ros::ok()&&is_thread_ok){
    ros::spinOnce();
    loop_rate.sleep();
    UpdateMessage(corners);
    pub1.publish(corners[0]);
    pub2.publish(corners[1]);
    pub3.publish(corners[2]);
    pub4.publish(corners[3]);
  }
  ros::shutdown();
  is_thread_ok = false;
  return 0;
}