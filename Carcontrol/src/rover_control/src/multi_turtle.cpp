#include <ros/ros.h>
#include <turtlesim/Spawn.h>
int main(int argc,char** argv){
    ros::init(argc, argv, "turtle_new");
    ros::NodeHandle node;
    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle_client = node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv_msg;
    srv_msg.request.x = 5.5;
    srv_msg.request.y = 5.5;
    srv_msg.request.theta = 0;
    srv_msg.request.name = "A";
    add_turtle_client.call(srv_msg);
    srv_msg.request.x = 5.5;
    srv_msg.request.y = 5.5;
    srv_msg.request.theta = 0;
    srv_msg.request.name = "B";
    add_turtle_client.call(srv_msg);
    srv_msg.request.x = 5.5;
    srv_msg.request.y = 5.5;
    srv_msg.request.theta = 0;
    srv_msg.request.name = "C";
    add_turtle_client.call(srv_msg);
    srv_msg.request.x = 5.5;
    srv_msg.request.y = 5.5;
    srv_msg.request.theta = 0;
    srv_msg.request.name = "D";
    add_turtle_client.call(srv_msg);
}