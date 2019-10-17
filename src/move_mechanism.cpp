#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>

constexpr int LOOP_RATE = 20;

ros::Rate *loop_rate;
ros::Publisher pub;
ros::Publisher mdd;
std_msgs::Int32 msg;
bool skip = true;
bool bath = false;
int count = 0;
int mode = 0;
int spread = 0;
int stm_status = 0;

inline void sendSerial(uint8_t id,uint8_t cmd,int16_t data);
inline void waitMove(int wait = 10);

void runFunc(const std_msgs::Int32 &data){
    if(data.data == 0){
        skip = true;
        msg.data = -1;
        pub.publish(msg);
        return;
    }else if(data.data > 0){
        mode = data.data;
        skip = false;
        ROS_INFO("moving:%d",mode);
    }
    msg.data = mode;
    switch (mode){
        case 1://タオル１
            //sendSerial(1,2,1);
            //for(int i = 0;i < 2;i++){
                waitMove(1);
                sendSerial(17,7,1);
                waitMove(10);
                sendSerial(17,6,1);
                waitMove(10);
                sendSerial(17,7,-1);
                waitMove(10);
                sendSerial(17,6,-1);
            //}
            break;
        case 2://タオル２
            //sendSerial(1,2,1);
            //for(int i = 0;i < 2;i++){
                waitMove(1);
                sendSerial(17,7,2);
                waitMove(10);
                sendSerial(17,6,2);
                waitMove(10);
                sendSerial(17,7,-2);
                waitMove(10);
                sendSerial(17,6,-2);
            //}
            loop_rate->sleep();
            sendSerial(1,2,0);
            break;
        case 3://シーツはじめ
            //sendSerial(1,5,2);
            //loop_rate->sleep();
            sendSerial(1,4,1);
            waitMove(10);
            sendSerial(1,5,1);
            waitMove(40);
            sendSerial(1,5,-1);
            break;
        case 4://シーツ終わり
            sendSerial(1,5,2);
            //waitMove(true);
            //sendSerial(1,5,-2);
            break;
        case 5://展開１
            if(spread != 1){
                sendSerial(1,3,1);
                //sendSerial(1,6,1);
                waitMove(10);
            }
            break;
        case 6://展開２
            if(spread != 2){
                sendSerial(1,3,2);
                //sendSerial(1,6,2);
                waitMove(10);
            }
            break;
        case 7://収納
            sendSerial(1,5,-2);
            sendSerial(1,4,0);
            loop_rate->sleep();
            loop_rate->sleep();
            if(spread == 2){
                sendSerial(1,3,1);
                //sendSerial(1,6,0);
            }
            waitMove(10);
            break;
        default:
            skip = true;
            msg.data = -1;
            pub.publish(msg);
            break;
    }
    skip = true;
    ROS_INFO("move Fin");
    pub.publish(msg);
    ros::spinOnce();
}

inline void waitMove(int wait){
    count = 0;
    while(ros::ok() && !skip){
        ros::spinOnce();
        loop_rate->sleep();
        count++;
        if(count < wait){
        }else if(stm_status == 0 ||( count > 80 && (mode == 1 || mode == 2))){
            count = 0;
            break;
        }
    }
}

void getStatus(const std_msgs::Int16 &data){
    stm_status = (data.data & 0xff);
    spread = (data.data >> 8) & 0xff;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "move_mechanism");
    ros::NodeHandle nh;
    loop_rate = new ros::Rate(LOOP_RATE);
    ros::Subscriber sub = nh.subscribe("run_mechanism",100,runFunc);
    ros::Subscriber wait = nh.subscribe("mechanism_status",100,getStatus);
    pub = nh.advertise<std_msgs::Int32>("mechanism_response",100);
    mdd = nh.advertise<std_msgs::Int32>("motor_serial",100);
    ROS_INFO("Move Mechanism start");
    while(ros::ok()){
        ros::spinOnce();
        loop_rate->sleep();
    }
    delete loop_rate;
    return 0;
}

inline void sendSerial(uint8_t id,uint8_t cmd,int16_t data){
    if(skip){
        return;
    }
    static std_msgs::Int32 send_data;
    send_data.data = (id << 24) + (cmd << 16) + (data & 0xffff);
    mdd.publish(send_data);
    ros::spinOnce();//絶対送信するマン
}
