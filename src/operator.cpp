#include <iostream>
#include <cstdio>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

ros::Publisher mdd;
inline void sendSerial(uint8_t,uint8_t,int16_t);

int main(int argc,char **argv){
    ros::init(argc,argv,"operator");
    ros::NodeHandle nh;
    char buf[10];
    int mode = false;
    std::cout << "select mode\n0 : send serial\n1 : send laundry node" << std::endl;
    std::cin >> buf;
    mode = std::atoi(buf);
    if(mode == 0){
        ROS_INFO("send to serial mode");
    }else{
        ROS_INFO("send to laundry node mode");
        mode = 1;
    }
    ros::Publisher moter = nh.advertise<std_msgs::Float32MultiArray>(mode ? "Operation" : "motor",100);
    mdd = nh.advertise<std_msgs::Int32>("motor_serial",100);
    std_msgs::Float32MultiArray msg;
    double cmd,x,y,yaw;
    msg.data.resize(4);
    while(ros::ok()){
        ros::spinOnce();
        std::cout << "cmd:";
        std::cin >> buf;
        cmd = std::atoi(buf);
        msg.data[0] = cmd;
        msg.layout.data_offset = cmd;
        if(cmd == -9){
            break;
        }
        if(cmd >= 0){
            std::cout << "X:";
            std::cin >> buf;
            x = std::atof(buf);
            msg.data[1] = x;
            std::cout << "Y:";
            std::cin >> buf;
            y = std::atof(buf);
            msg.data[2] = y;
            std::cout << "Yaw:";
            std::cin >> buf;
            yaw = std::atof(buf);
            msg.data[3] = yaw;
        }
        ROS_INFO("%f : %f %f %f",msg.data[0],msg.data[1],msg.data[2],msg.data[3]);
        if(cmd == 21){
            sendSerial(x,y,yaw);
        }else{
            moter.publish(msg);
        }
    }
    return 0;
}

inline void sendSerial(uint8_t id,uint8_t cmd,int16_t data){
    static std_msgs::Int32 send_data;
    send_data.data = (id << 24) + (cmd << 16) + (data & 0xffff);
    mdd.publish(send_data);
    ros::spinOnce();//絶対送信するマン
}
