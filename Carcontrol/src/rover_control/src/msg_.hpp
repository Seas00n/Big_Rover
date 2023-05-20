#include <iostream>
#include <cmath>
#include "spine/socketcan.h"
#include "spine/dynamixel_motor.h"
#include "spine/minicheetah_motor.h"
#include "rover_control/corner.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "damp_.hpp"
static const float P_MIN = -12.5f;
static const float P_MAX = 12.5f;
static const float V_MIN = -30.0f;
static const float V_MAX = 30.0f;
static const float I_MAX = 18.0f;


class SteeringWheelTypeDef{
    public:
        float wheel_p_actual;
        float wheel_p_desired;
        float wheel_v_actual;
        float wheel_v_desired;
        float steer_p_actual;
        float steer_p_desired;
        rover_control::corner msg;
        ros::Publisher msg_pub;
        SteeringWheelTypeDef(){
            wheel_p_actual = 0;
            wheel_p_desired = 0;
            wheel_v_actual = 0;
            wheel_v_desired = 0;
            steer_p_actual = 0;
            steer_p_desired = 0;
        }
        void setPublisher(ros::NodeHandle& nh,std::string name){
            msg_pub = nh.advertise<rover_control::corner>(name,100);
        }
        void MsgPublish(){
            msg.steer_pos_actual = steer_p_actual;
            msg.steer_pos_desired = steer_p_desired;
            msg.wheel_vel_actual = wheel_v_actual;
            msg.wheel_vel_desired = wheel_v_desired;
            msg_pub.publish(msg);
        }
        ~SteeringWheelTypeDef(){
            std::cout<<"free"<<std::endl;
        }
};


class RoverTypeDef{
    public:
        float rover_v;
        float rover_w;
        float rover_px_desired;
        float rover_py_desired;
        float rover_theta_desired;
        float rover_px_actual;
        float rover_py_actual;
        float rover_theta_actual;
        int rover_motion_state = 0;
        RoverTypeDef(){
            rover_v = 0;
            rover_w = 0;
            rover_motion_state = 0;
            rover_px_desired = 0;
            rover_py_desired = 0;
            rover_theta_desired = 0;
            rover_px_actual = 0;
            rover_py_actual = 0;
            rover_theta_actual = 0;

        };
        ~RoverTypeDef(){
            std::cout<<"free rover"<<std::endl;
        };
};


//用于debug
std::vector<ros::Publisher> debug_turtle_pubs;
std::vector<SecondOrderDynamics> steer_dynamics;
ros::Subscriber sub_turtle1;
extern RoverTypeDef rover_temp;


void thread_sleep(int ms){
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
int depackMsg(struct can_frame & frame, std::vector<SteeringWheelTypeDef>& actuators){
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
    if(id==0||id==1||id==2||id==3){
        actuators[id].wheel_p_actual = p;
        actuators[id].wheel_v_actual = v;
    }
    return 1;
}


void turtle1Odm(const turtlesim::Pose::ConstPtr& p, RoverTypeDef& rover_temp){
    rover_temp.rover_px_actual = p->x;
    rover_temp.rover_py_actual = p->y;
    rover_temp.rover_theta_actual = p->theta;
}


void setDebugTurtlePosition(std::vector<double>target_x,std::vector<double> target_y,int step){
    ros::Rate loop_rate(100);
    thread_sleep(2000);
    for(int j=0;j<4;j++){
        SecondOrderDynamics x_dynamics = SecondOrderDynamics(5.0,1,0,5.5,0.01);
        SecondOrderDynamics y_dynamics = SecondOrderDynamics(5.0,1,0,5.5,0.01);
        geometry_msgs::Twist msg;
        for(int i=0;(i<step&&ros::ok());i++){
            ros::spinOnce();
            x_dynamics.Update(target_x[j]);
            y_dynamics.Update(target_y[j]);
            msg.linear.x = x_dynamics.get_yd();
            msg.linear.y = y_dynamics.get_yd();
            debug_turtle_pubs[j+1].publish(msg);
            loop_rate.sleep();
        }
        x_dynamics.~SecondOrderDynamics();
        y_dynamics.~SecondOrderDynamics();
    }
}


void InitialDebugTurtle(ros::NodeHandle nh){
    //数据显示
    debug_turtle_pubs.push_back(nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10));
    debug_turtle_pubs.push_back(nh.advertise<geometry_msgs::Twist>("/A/cmd_vel",10));
    debug_turtle_pubs.push_back(nh.advertise<geometry_msgs::Twist>("/B/cmd_vel",10));
    debug_turtle_pubs.push_back(nh.advertise<geometry_msgs::Twist>("/C/cmd_vel",10));
    debug_turtle_pubs.push_back(nh.advertise<geometry_msgs::Twist>("/D/cmd_vel",10)); 
    //用二阶系统模仿输入位置指令后舵机的速度变化
    steer_dynamics.push_back(SecondOrderDynamics(5.0,1.2,0.0,0.0,0.01));
    steer_dynamics.push_back(SecondOrderDynamics(5.0,1.2,0.0,0.0,0.01));
    steer_dynamics.push_back(SecondOrderDynamics(5.0,1.2,0.0,0.0,0.01));
    steer_dynamics.push_back(SecondOrderDynamics(5.0,1.2,0.0,0.0,0.01));
    //订阅turtle1的Pose作为仿真中小车的位姿
    sub_turtle1 = nh.subscribe<turtlesim::Pose>("/turtle1/pose",100,boost::bind(&turtle1Odm,_1,boost::ref(rover_temp)));
    thread_sleep(10);
    std::vector<double> target_x = {6,6,5,5};
    std::vector<double> target_y = {5,6,5,6};
    setDebugTurtlePosition(target_x,target_y,150);
}

void vIW_W(double vIR_Wx,double vIR_Wy, double wIR_W, double rRS_Wx,double rRS_Wy, geometry_msgs::Twist& msg){
    double vx = vIR_Wx-rRS_Wy*wIR_W;
    double vy = vIR_Wy+rRS_Wx*wIR_W;
    msg.linear.x = sqrt(vx*vx+vy*vy);
}
void DebugTurtleDynamics(RoverTypeDef& rover_temp,std::vector<SteeringWheelTypeDef>& actuators){
    std::cout<<"rover_temp in main"<<&(rover_temp)<<std::endl;
    geometry_msgs::Twist turtle1_twist;
    turtle1_twist.linear.x = rover_temp.rover_v;
    turtle1_twist.angular.z = rover_temp.rover_w;
    std::cout<<"rover_v:"<<rover_temp.rover_v<<std::endl;
    std::cout<<"rover_w:"<<rover_temp.rover_w<<std::endl;
    debug_turtle_pubs[0].publish(turtle1_twist);
    geometry_msgs::Twist wheel_twist;
    double r_theta = rover_temp.rover_theta_actual;
    std::cout<<"r_theta"<<r_theta<<std::endl;
    double vIR_Wx = rover_temp.rover_v*cos(r_theta);
    double vIR_Wy = rover_temp.rover_v*sin(r_theta);
    double wIR_W = rover_temp.rover_w;
    double rRS_Wx = cos(r_theta)*0.5-sin(r_theta)*0.5;
    double rRS_Wy = sin(r_theta)*0.5+cos(r_theta)*0.5;
    vIW_W(vIR_Wx,vIR_Wy,wIR_W,rRS_Wx,rRS_Wy,wheel_twist);
    steer_dynamics[0].Update(actuators[0].steer_p_desired+r_theta);
    actuators[0].steer_p_actual = steer_dynamics[0].get_y();
    wheel_twist.angular.z = steer_dynamics[0].get_yd();
    debug_turtle_pubs[1].publish(wheel_twist);

    rRS_Wx = cos(r_theta)*0.5-sin(r_theta)*0.5;
    rRS_Wy = sin(r_theta)*0.5+cos(r_theta)*0.5;
    vIW_W(vIR_Wx,vIR_Wy,wIR_W,rRS_Wx,rRS_Wy,wheel_twist);
    steer_dynamics[1].Update(actuators[1].steer_p_desired+r_theta);
    actuators[1].steer_p_actual = steer_dynamics[1].get_y();
    wheel_twist.angular.z = steer_dynamics[1].get_yd();
    debug_turtle_pubs[2].publish(wheel_twist);

    rRS_Wx = -cos(r_theta)*0.5-sin(r_theta)*0.5;
    rRS_Wy = sin(r_theta)*0.5+cos(r_theta)*0.5;
    vIW_W(vIR_Wx,vIR_Wy,wIR_W,rRS_Wx,rRS_Wy,wheel_twist);
    steer_dynamics[2].Update(actuators[2].steer_p_desired+r_theta);
    actuators[2].steer_p_actual = steer_dynamics[2].get_y();
    wheel_twist.angular.z = steer_dynamics[2].get_yd();
    debug_turtle_pubs[3].publish(wheel_twist);

    rRS_Wx = cos(r_theta)*0.5-sin(r_theta)*0.5;
    rRS_Wy = -sin(r_theta)*0.5-cos(r_theta)*0.5;
    vIW_W(vIR_Wx,vIR_Wy,wIR_W,rRS_Wx,rRS_Wy,wheel_twist);
    steer_dynamics[3].Update(actuators[3].steer_p_desired+r_theta);
    actuators[3].steer_p_actual = steer_dynamics[3].get_y();
    wheel_twist.angular.z = steer_dynamics[3].get_yd();
    debug_turtle_pubs[4].publish(wheel_twist);
}
