#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

std_msgs::Float32MultiArray msg;

int main(int argc, char **argv){
    ros::init(argc, argv, "dummy_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("place",100);
    char buf[10];
    msg.data.resize(8);
    while(ros::ok()){
        ros::spinOnce();
        std::cout << "X:";
        std::cin >> buf;
        msg.data[0] = std::atof(buf);
        std::cout << "Y:";
        std::cin >> buf;
        msg.data[1] = std::atof(buf);
        std::cout << "T:";
        std::cin >> buf;
        msg.data[2] = std::atof(buf);
        std::cout << "Yaw:";
        std::cin >> buf;
        msg.data[3] = std::atof(buf);
        std::cout << "Vx:";
        std::cin >> buf;
        msg.data[4] = std::atof(buf);
        std::cout << "Vy:";
        std::cin >> buf;
        msg.data[5] = std::atof(buf);
        std::cout << "Vt:";
        std::cin >> buf;
        msg.data[6] = std::atof(buf);
        std::cout << "status:";
        std::cin >> buf;
        msg.data[7] = std::atof(buf);
        ROS_INFO("X:%d,Y:%d,T:%d,Yaw:%d,Vx:%d,Vy:%d,Vt:%d,status:%d",(int)msg.data[0],(int)msg.data[1],(int)msg.data[2],(int)msg.data[3],(int)msg.data[4],(int)msg.data[5],(int)msg.data[6],(int)msg.data[7]);
        pub.publish(msg);
    }
    return 0;
}

