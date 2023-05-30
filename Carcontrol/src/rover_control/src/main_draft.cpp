#include <iostream>
#include <math.h>
#include "spine/socketcan.h"
#include "spine/dynamixel_motor.h"
#include "spine/minicheetah_motor.h"
#include "msg_.hpp"
// #include "rover_control/corner.h"
#include "rover_control/rover.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

std::vector<SteeringWheelTypeDef> actuators;
RoverTypeDef rover_temp;

bool is_thread_ok = true;

#define PI 3.141592657
#define wheel_to_center_x 0.5
#define wheel_to_center_y 0.5
#define wheel_to_center sqrt(pow(wheel_to_center_x,2)+pow(wheel_to_center_y,2))
#define cal_distance(x,y) sqrt(pow(x,2)+pow(y,2))
enum CAR_STATE{GO_AHEAD=0,CHANGE_TO_TURN=1,TURN=2,CHANGE_TO_GO=3};



void canRxThread(SocketCan *sc){
  sc->clearRxCallback();
  struct can_frame frame{};
  while(is_thread_ok){
    *sc >> frame;
    if(depackMsg(frame,actuators)){
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
        steering_Motor[i].setQDes(actuators[i].steer_p_desired);
        wheel_Motor[i].setQdDes(actuators[i].wheel_v_desired);
        wheel_Motor[i].setKpKd(0,4);
        steering_Motor[i].control();
        thread_sleep(5);
        wheel_Motor[i].control();
        thread_sleep(5);
      }
    }
}


void FSMControl(const rover_control::rover::ConstPtr& p, RoverTypeDef& rover_temp,
                std::vector<SteeringWheelTypeDef>& actuators){
  volatile int current_state = rover_temp.rover_motion_state;
  // std::cout<<"current state:"<<current_state<<std::endl;
  if(current_state==0){
    // std::cout<<"State GO_AHEAD";
    rover_temp.rover_v= p->rover_vx;
    rover_temp.rover_w = 0;
    for(int i=0;i<4;i++){
      actuators[i].wheel_v_desired = rover_temp.rover_v;
      actuators[i].steer_p_desired = 0;
    }
    // std::cout<<rover_temp.rover_v<<std::endl;
    if((abs(p->rover_w)<0.01)&&abs(p->rover_vx<0.01)&&(p->rover_state==CHANGE_TO_TURN)){
      rover_temp.rover_motion_state = CHANGE_TO_TURN;
      // std::cout<<"CHANGE next state"<<rover_temp.rover_motion_state<<std::endl;
    }
    return;
  }
  if (current_state ==1){
    // std::cout<<"State Change to Turn"<<std::endl;
    rover_temp.rover_v = 0;
    rover_temp.rover_w = 0;
    actuators[0].steer_p_desired = -135.0*PI/180.0;
    actuators[1].steer_p_desired = -45.0*PI/180.0;
    actuators[2].steer_p_desired = 135.0*PI/180.0;
    actuators[3].steer_p_desired = 45.0*PI/180.0;
    if(p->rover_state==TURN){
      rover_temp.rover_motion_state = TURN;
      // std::cout<<"CHANGE next state"<<rover_temp.rover_motion_state<<std::endl;
    }
    return;
  }
  if(current_state == 2){
    // std::cout<<"State Turn"<<p->rover_w<<std::endl;
    rover_temp.rover_v = 0;
    rover_temp.rover_w = p->rover_w;
    double wheel_turn = rover_temp.rover_w*wheel_to_center;
    for(int i=0;i<4;i++){
      actuators[i].wheel_v_desired = wheel_turn;
    }
    // std::cout<<rover_temp.rover_w<<std::endl;
    actuators[0].steer_p_desired = -135.0*PI/180.0;
    actuators[1].steer_p_desired = -45.0*PI/180.0;
    actuators[2].steer_p_desired = 135.0*PI/180.0;
    actuators[3].steer_p_desired = 45.0*PI/180.0;
    if(abs(p->rover_w)<0.01&&abs(p->rover_vx)<0.01&&p->rover_state==CHANGE_TO_GO){
      rover_temp.rover_motion_state = CHANGE_TO_GO;
      // std::cout<<"CHANGE next state"<<rover_temp.rover_motion_state<<std::endl;
    }
    return;
  }
  if(current_state == 3){
    // std::cout<<"State Change to GO"<<std::endl;
    rover_temp.rover_v = 0;
    rover_temp.rover_w = 0;
    for(int i=0;i<4;i++){
      actuators[i].steer_p_desired = 0;
    }
    if(p->rover_state==GO_AHEAD){
      rover_temp.rover_motion_state = GO_AHEAD;
      // std::cout<<"CHANGE next state"<<rover_temp.rover_motion_state<<std::endl;
    }
    return;
  }
}




void HardWareInitialization(std::vector<DynamixelMotor>& steering_Motor, std::vector<AK10_9Motor>& wheel_Motor){
  SocketCan can("can0",1e6);
  steering_Motor.push_back(DynamixelMotor(1,"/dev/ttyUSB0"));
  steering_Motor.push_back(DynamixelMotor(2,"/dev/ttyUSB0"));
  steering_Motor.push_back(DynamixelMotor(3,"/dev/ttyUSB0"));
  steering_Motor.push_back(DynamixelMotor(4,"/dev/ttyUSB0"));
  thread_sleep(5);
  wheel_Motor.push_back(AK10_9Motor());
  wheel_Motor.push_back(AK10_9Motor());
  wheel_Motor.push_back(AK10_9Motor());
  wheel_Motor.push_back(AK10_9Motor());
  wheel_Motor[0].attach(1,&can);thread_sleep(5);
  wheel_Motor[1].attach(2,&can);thread_sleep(5);
  wheel_Motor[2].attach(3,&can);thread_sleep(5);
  wheel_Motor[3].attach(4,&can);thread_sleep(5);
}
void SoftwareInitialization(std::vector<SteeringWheelTypeDef>& steeringwheels,
                            ros::NodeHandle& nh){
  for(int i=0;i<4;i++){
    std::string topic_name = "steerwheel"+i;
    steeringwheels.push_back(SteeringWheelTypeDef());
    steeringwheels[i].setPublisher(nh,topic_name);
  }
}


int main(int argc, char *argv[]){
  // std::vector<DynamixelMotor> steering_Motor;//舵机硬件抽象
  // std::vector<AK10_9Motor> wheel_Motor;//车轮硬件抽象
  // HardWareInitialization(steering_Motor,wheel_Motor);


  setlocale(LC_ALL, "");
  ros::init(argc, argv, "control");
  ros::NodeHandle nh;  
  ros::Rate loop_rate(100);
  ROS_INFO_STREAM("节点初始化完成");
  SoftwareInitialization(actuators,nh);

  //订阅手柄发布的无人车状态消息
  ros::Subscriber sub = nh.subscribe<rover_control::rover>("rover_state",1000,boost::bind(&FSMControl,_1,
                                                            boost::ref(rover_temp),boost::ref(actuators)));



  InitialDebugTurtle(nh);
  // std::thread canRx(canRxThread, &can);
  // canRx.detach();

  // std::thread cmdTx(cmdTxThread, wheel_Motor,steering_Motor);
  // cmdTx.detach();


  while(ros::ok()&&is_thread_ok){
    ros::spinOnce();
    DebugTurtleDynamics(rover_temp, actuators);
    for(int i=0;i<4;i++){
      actuators[i].MsgPublish();
    }
    loop_rate.sleep();
  }
  ros::shutdown();
  is_thread_ok = false;
  return 0;
}