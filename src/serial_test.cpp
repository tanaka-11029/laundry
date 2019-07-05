#include <iostream>
#include <cstdio>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

std_msgs::Float32MultiArray msg;

int main(int argc,char **argv){
    ros::init(argc,argv,"serial_test");
    ros::NodeHandle nh;
    ros::Publisher moter = nh.advertise<std_msgs::Float32MultiArray>("moter",100);
    double cmd,speed;
    msg.data.resize(4);
    while(ros::ok()){
        ros::spinOnce();
        std::cout << "cmd:";
        std::cin >> cmd;
        msg.data[0] = cmd;
        if(cmd == -2){
            break;
        }
        if(cmd >= 0){
            std::cout << "X:";
            std::cin >> speed;
            msg.data[1] = speed;
            std::cout << "Y:";
            std::cin >> speed;
            msg.data[2] = speed;
            std::cout << "Yaw:";
            std::cin >> speed;
            msg.data[3] = speed;
        }
        ROS_INFO("%f : %f %f %f",msg.data[0],msg.data[1],msg.data[2],msg.data[3]);
        moter.publish(msg);
    }
    return 0;
}
