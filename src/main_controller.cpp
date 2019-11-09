#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <RasPiDS3.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <laundry/PixyData.h>
#include <cs_connection/PrintStatus.h>
#include <cs_connection/RsDataMsg.h>

#define DS3 false

//using std::fabs;

constexpr int LOOP_RATE  = 25;
constexpr double AMAX[3] = {1000,1000,300}; // mm/s/s
constexpr double FAST_V = 1800;
constexpr double JARK[3] = {100,100,100}; //mm/s/s/s
constexpr double VMAX[3] = {1200,1200,600}; // mm/s
std_msgs::Float32MultiArray move_data;
cs_connection::PrintStatus send_status;
ros::Publisher motor;
ros::Publisher mechanism;
ros::Publisher mdd;
bool coat = false;//0:赤　1:青
bool fight = false;//0:予選 1:決勝
bool skip = false;
bool auto_move = false;
bool warn = false;
bool rs_disconnect = false;
bool towel[2] = {true,true};
bool seats = true;
bool stm_auto = false;
bool change_data = true;
bool clip_limit = false;
bool limit_right = false;
bool limit_left = false;
bool emergency = false;
bool rs_data = false;
bool rs_use = false;
bool ready = false;
bool fast_mode = false;
bool stopped = false;
bool sheet_first = true;
bool towel_switch = false;
int spreaded = 0;
int stm_status = 0;
int status = 0;
int next_move = 0;
int warn_count = 0;
int rs_count = 0;
int wait_num = 0;
int lidar_x = 0;
int lidar_y = 0;
double rsy_diff = 0;
double rsx_diff = 0;
double rs_goal_y = 0;
double rs_goal_x = 0;
double tmp_goal_y = 0;
double tmp_goal_x = 0;
double goal_x,goal_y,goal_yaw;
double now_v_x,now_v_y,now_omega;
double now_x,now_y,now_yaw;
double rs_data_x,rs_data_y,rs_data_z;
FILE* fp;
#if DS3
RPDS3::DualShock3 controller;
#endif

inline double constrain(double x,double a,double b){
    return (x < a ? a : x > b ? b : x );
}

void operate(const std_msgs::Float32MultiArray &data);
inline void autoMove();
inline void sendSerial(uint8_t id,uint8_t cmd,int16_t data);
inline void cmdnum(double cmd,double x,double y,double omega);
inline void cmd1(int data);
inline void setAuto(double x,double y,double yaw);
inline void sendMechanism(int);

void getData(const std_msgs::Float32MultiArray &place){
    now_x = place.data[0];
    now_y = place.data[1];
    now_yaw = place.data[3];
    now_v_x = place.data[4];
    now_v_y = place.data[5];
    now_omega = place.data[6];
    stm_status = (int)place.data[7];
    if(warn_count != 0){
        warn_count = 0;
    }
}

void getResponse(const std_msgs::Int32 &data){
    if(data.data == wait_num){
        wait_num = 0;
        if(status == 0 && !auto_move){
            status = next_move;
        }
        switch (data.data){
            case 1://タオル１
                if(towel_switch){
                    towel[1] = false;
                }else{
                    towel[0] = false;
                }
                break;
            case 2://タオル２
                if(towel_switch){
                    towel[0] = false;
                }else{
                    towel[1] = false;
                }
                break;
            case 3://シーツ始め
                break;
            case 4://シーツ終わり
                seats = false;
                break;
            case 5://第一展開
                //spreaded = 1;
                break;
            case 6://第二展開
                //spreaded = 2;
                break;
            case 7://収納
                //spreaded = 0;
                break;
        }
    }else if(data.data == -1){
        wait_num = 0;
    }
}

void getRsmsg(const cs_connection::RsDataMsg &data){
    rs_data_x = data.x_distance;
    rs_data_y = data.y_distance;
    rs_data_z = data.z_distance;
    if(rs_use){
        rs_data = true;
        rsy_diff = rs_data_y - rs_goal_y;
        tmp_goal_y = now_y + (rsy_diff * 0.8);
        rsx_diff = rs_data_x - rs_goal_x;
        tmp_goal_x = now_x + (rsx_diff * 0.9);
    }
    if(rs_count != 0){
        rs_count = 0;
    }
}

void getSwitch(const std_msgs::Int8 &data){
    static bool last_emergency = false;
    static int count;
    limit_left = (data.data >> 2) & 0x01;
    limit_right = (data.data >> 1) & 0x01;
    emergency = (data.data) & 0x01;
    if(emergency != last_emergency){
        last_emergency = emergency;
        count = 0;
        sendSerial(1,7,emergency);
        std_msgs::Float32MultiArray msg;
        msg.data.resize(1);
        if(emergency){
            msg.data[0] = -2;
            operate(msg);
        }else if(stopped){
            msg.data[0] = -3;
            operate(msg);
        }
    }else if(count < 10){
        count ++;
        sendSerial(1,7,emergency);
    }
}

void getStart(const std_msgs::Bool &data){//スタートスイッチで始める。
    if(ready && data.data && !emergency){
        std_msgs::Float32MultiArray msg;
        msg.data.resize(1);
        msg.data[0] = -5;
        operate(msg);
    }
}

void getSpread(const std_msgs::Int16 &data){
    spreaded = (data.data >> 8) & 0xff;
}

void getLimit(const std_msgs::Bool &data){
    clip_limit = data.data;
}

void getLidar(const std_msgs::Int64 &data){
    lidar_x = data.data & 0xffffffff;
    lidar_y = (data.data >> 32) & 0xffffffff;
}

int main(int argc,char **argv){
    constexpr double NOMAL_Y = 5700;
    constexpr double SEATS_Y = 6740;
    constexpr double TOWEL_Y = 5240;
    constexpr double point[9][2][3] = {//赤　青
        {{500,NOMAL_Y,0},{-500,NOMAL_Y,0}},//ポール間に入る前
        {{1800,NOMAL_Y,0},{-1800,NOMAL_Y,0}},//ポール間に入る
        {{1680,SEATS_Y - 300,0},{-3740,SEATS_Y - 300,0}},//シーツかけ始め 1830
        {{3760,SEATS_Y,0},{-1650,SEATS_Y,0}},//シーツかけ終わり
        {{1950,TOWEL_Y,0},{-2020,TOWEL_Y,0}},//タオル１予選４1980 2750 2000
        {{1920,TOWEL_Y,0},{-2130,TOWEL_Y,0}},//タオル１決勝 2180 2090
        {{3140,TOWEL_Y,0},{-3190,TOWEL_Y,0}},//タオル２予選６3300 3550 3020
        {{3280,TOWEL_Y,0},{-3480,TOWEL_Y,0}},//タオル２決勝 3500 3410
        {{200,NOMAL_Y,0},{-200,NOMAL_Y,0}}
    };
    //rs_x,rs_y,lidar_fence,lidar_base,lidar_y
    constexpr double offset[6][2][5] = {//赤　青
        {{816 ,3750,2865,1800,2950},{-878,3750,1990,315 ,2930}},//タオル１　予選 2870 340
        {{680 ,3750,2750,1710,2950},{-710,3750,2130,390 ,2930}},//タオル１　決勝 3780
        {{-420,3770,1690,670 ,2950},{422 ,3770,3200,1455,2950}},//タオル２　予選 3190 1430
        {{-695,3770,1430,390 ,2950},{660 ,3770,3470,1695,2950}},//タオル２　決勝
        {{1010,2030,3110,2060,1430},{1018,2030,3880,2060,1430}},//シーツ 始め 3180 1370
        {{-1050,2030,1075,0,1430},{-1000,2030,1800,0,1430}}//シーツ　終わり /2080
    };
    constexpr char coat_name[2][10] = {
        {"赤"},
        {"青"}
    };
    constexpr char fight_name[2][10] = {
        {"予選"},
        {"決勝"}
    };
    ros::init(argc, argv, "main_controller");
    ros::NodeHandle nh;
    motor = nh.advertise<std_msgs::Float32MultiArray>("motor",100);
    mdd = nh.advertise<std_msgs::Int32>("motor_serial",100);
    mechanism = nh.advertise<std_msgs::Int32>("run_mechanism",100);
    ros::Publisher gui_status = nh.advertise<cs_connection::PrintStatus>("print_status",100);
    ros::Subscriber place = nh.subscribe("place",100,getData);
    ros::Subscriber operation = nh.subscribe("Operation",100,operate);
    ros::Subscriber mechanism_response = nh.subscribe("mechanism_response",100,getResponse);
    ros::Subscriber rs_sub = nh.subscribe("rs_msg",100,getRsmsg);
    //ros::Subscriber limit_sub = nh.subscribe("clip_limit",100,getLimit);
    ros::Subscriber spread_sub = nh.subscribe("mechanism_status",100,getSpread);
    ros::Subscriber lidar_sub = nh.subscribe("lidar",100,getLidar);
    ros::Subscriber switch_sub = nh.subscribe("switch",100,getSwitch);
    ros::Subscriber start_sub = nh.subscribe("start_switch",100,getStart);
    ros::Rate loop_rate(LOOP_RATE);
#if DS3
    controller.yReverseSet(true);
#endif
    int last_status = status;
    int last_next = next_move;
    int count = 0;
    int count_towel = 0;
    int count_hang = 0;
    int num = 0;
    int lidar_x_diff = 0;
    int lidar_y_diff = 0;
    double speed = 2;
    bool last_seats;
    bool last_towel[2];
    bool bath = false;
    bool zero = false;
    bool moter4 = false;
    bool send_flag = false;
    bool lidar_x_ok = false;
    bool towel_x_ok = false;
    bool towel_y_ok = false;
    bool towel_back = false;
    bool sheet_x_ok = false;
    char buf[100],filename[100];
    time_t now_t = time(nullptr);
    time_t start_t;
    tm* local_time = localtime(&now_t);
    std::cout << "コート選択\n0 : 赤\t1 : 青" << std::endl;
    std::cin >> buf;
    coat = std::atoi(buf);
    ROS_INFO("coat : %d",coat);
    std::cout << "モード選択\n0 : 予選\t1 : 決勝" << std::endl;
    std::cin >> buf;
    fight = std::atoi(buf);
    ROS_INFO("fight : %d",fight);
    ROS_INFO("laundry_node start");
    sprintf(filename,"/home/tanaka/2019robocon/src/laundry/log/%d_%d_%d:%d.txt",local_time->tm_mon+1,local_time->tm_mday,local_time->tm_hour,local_time->tm_min);
    fp = fopen(filename,"w");
    if(fp == nullptr){
        ROS_ERROR("faild to open log file");
    }else{
        ROS_INFO("opened log file %s",filename);
    }
    while(ros::ok()
#if DS3
            && !(controller.button(RPDS3::START) && controller.button(RPDS3::RIGHT))
#endif
         ){
#if DS3
        controller.update();
        if(controller.button(RPDS3::R1)){
            if(controller.press(RPDS3::SELECT)){
                cmd1(-1);
                sendSerial(15,255,0);
            }
            if(controller.press(RPDS3::UP)){
                sendSerial(15,4,100);
                moter4 = true;
            }else if(controller.press(RPDS3::DOWN)){
                sendSerial(15,4,-100);
                moter4 = true;
            }else if(controller.release(RPDS3::UP) || controller.release(RPDS3::DOWN)){
                sendSerial(15,4,0);
                if(moter4){
                    sendSerial(15,4,0);
                    moter4 = false;
                }else{
                    sendSerial(15,2,0);
                }
            }
        }else{
            if(controller.press(RPDS3::SELECT)){
                cmd1(-2);
                sendSerial(15,255,0);
            }
            if(controller.press(RPDS3::UP)){
                sendSerial(15,2,100);
            }else if(controller.press(RPDS3::DOWN)){
                sendSerial(15,2,-100);
            }else if(controller.release(RPDS3::UP) || controller.release(RPDS3::DOWN)){
                if(moter4){
                    sendSerial(15,4,0);
                    moter4 = false;
                }else{
                    sendSerial(15,2,0);
                }
            }
        }
        if(controller.press(RPDS3::RIGHT) && !controller.button(RPDS3::START)){
            sendSerial(15,3,-100);
        }else if(controller.press(RPDS3::LEFT)){
            sendSerial(15,3,250);
        }else if(controller.release(RPDS3::LEFT) || controller.release(RPDS3::RIGHT)){
            sendSerial(15,3,0);
        }

        if(controller.press(RPDS3::SQUARE)){//ソレノイド
            sendSerial(15,10,1);
        }else if(controller.release(RPDS3::SQUARE)){
            sendSerial(15,10,-1);
        }
        if(controller.press(RPDS3::CIRCLE)){
            sendSerial(15,10,2);
        }else if(controller.release(RPDS3::CIRCLE)){
            sendSerial(15,10,-2);
        }
        if(controller.press(RPDS3::TRIANGLE)){
            sendSerial(1,5,1);
        }else if(controller.release(RPDS3::TRIANGLE)){
            sendSerial(1,5,-1);
        }
        if(controller.press(RPDS3::CROSS)){
            sendSerial(1,5,2);
        }else if(controller.release(RPDS3::CROSS)){
            sendSerial(1,5,-2);
        }

        if(controller.press(RPDS3::L1)){
            speed = 1;
        }else if(controller.release(RPDS3::L1)){
            speed = 2;
        }
        double left_x = controller.stick(RPDS3::LEFT_X);
        double left_y = controller.stick(RPDS3::LEFT_Y);
        double left_t = controller.stick(RPDS3::LEFT_T);
        double right_t = controller.stick(RPDS3::RIGHT_T);
        if(left_x != 0 || left_y !=  0|| left_t != 0 || right_t != 0){
            cmdnum(0,2*speed*left_x,2*speed*left_y,speed*(right_t - left_t)/300);
            if(!zero){
                zero = true;
            }
        }else if(zero){
            cmdnum(0,0,0,0);
            zero = false;
        }
#endif
        if(warn_count >= LOOP_RATE*30){
            if(!warn){
                if(status != 0 || next_move != 0){
                    fprintf(fp,"STM32通信切れ\n");
                }
                ROS_WARN("UNCONNECTED TO STM32");
                std_msgs::Float32MultiArray msg;
                msg.data.resize(1);
                msg.data[0] = -2;
                operate(msg);
                warn = true;
                change_data = true;
            }
        }else{
            warn_count++;
            if(warn){
                if(status != 0 || next_move != 0){
                    fprintf(fp,"STM32通信回復\n");
                }
                ROS_INFO("CONNECTED TO STM32");
                warn = false;
                change_data = true;
            }
        }
        if(rs_count >= LOOP_RATE*10){
            if(!rs_disconnect){
                if(status != 0 || next_move != 0){
                    fprintf(fp,"CS通信切れ\n");
                }
                ROS_WARN("UNCONNECTED TO CONTROL STATION");
                rs_data_x = 0;
                rs_data_y = 0;
                rs_data_z = 0;
                rsx_diff = 50000;
                rsy_diff = 50000;
                rs_disconnect = true;
            }
        }else{
            rs_count++;
            if(rs_disconnect){
                if(status != 0 || next_move != 0){
                    fprintf(fp,"CS通信回復\n");
                }
                ROS_INFO("CONNECTED TO CONTROL STATION");
                rs_disconnect = false;
            }
        }
        if(auto_move){
            if(stm_auto){
                if(stm_status == 0 || skip){
                    cmd1(-2);
                    auto_move = false;
                    if(wait_num == 0){
                        status = next_move;
                    }
                }
            }else{
                autoMove();
            }
        }
        if(status != last_status || next_move != last_next || seats != last_seats
                || towel[0] != last_towel[0] || towel[1] != last_towel[1] || change_data){
            ROS_INFO("status : %d\tnext : %d\tspread : %d",status,next_move,spreaded);
            change_data = false;
            last_status = status;
            last_next = next_move;
            last_seats = seats;
            last_towel[0] = towel[0];
            last_towel[1] = towel[1];
            send_status.status = status;
            send_status.next = next_move;
            send_status.coat = coat;
            send_status.fight = fight;
            send_status.seat = seats;
            send_status.towel1 = towel[0];
            send_status.towel2 = towel[1];
            send_status.warn = warn;
            gui_status.publish(send_status);
            ROS_INFO("now(%f,%f,%f)",now_x,now_y,now_yaw);
            ROS_INFO("goal(%f,%f,%f)", goal_x,goal_y,goal_yaw);
        }
        switch (status){
            case 0:
                break;
            case 1:
                now_t = time(nullptr);
                start_t = now_t;
                local_time = localtime(&now_t);
                fprintf(fp,"%d月%d日%d時%d分\t%s %s\n",local_time->tm_mon+1,local_time->tm_mday,local_time->tm_hour,local_time->tm_min,coat_name[coat],fight_name[fight]);
                //sendSerial(1,5,2);//シーツの端をつかむ
                if(fabs(now_x) >= 1500 && fabs(now_y) > 5200){
                    status = 3;
                    break;
                }
                stm_auto = false;
                //cmdnum(5,coat,VMAX[spreaded],AMAX[spreaded]);
                setAuto(point[0][coat][0],point[0][coat][1],point[0][coat][2]);
                fast_mode = true;//竿までの移動を早く
                status = 2;//pixyを使わない時
                next_move = 2;
                send_flag = false;
                break;
            case 2://ポール間に移動
                if(now_y > (NOMAL_Y - 700)){
                    stm_auto = false;
                    setAuto(point[1][coat][0],point[1][coat][1],point[1][coat][2]);
                    fast_mode = false;//微調整は遅めに
                    next_move = 3;
                    status = 3;//pixyを使わない時
                }else if(fight == 1 && fabs(now_yaw) < 2){
                    lidar_x_diff = lidar_x - 390;
                    if(fabs(lidar_x_diff) < 100){
                        goal_x = now_x + lidar_x_diff;
                    }
                }
                /*else if(now_y > (NOMAL_Y - 500) && !send_flag){
                    //cmdnum(5,-1,VMAX[spreaded],AMAX[spreaded]);
                    setAuto((coat ? -200 : 200),NOMAL_Y,0);
                    send_flag = true;
                }*/
                break;
            case 3://展開動作
                /*if(fabs(now_x) < 1400){
                    break;
                }*/
                if(spreaded != 1 && wait_num == 0){
                    now_t = time(nullptr);
                    fprintf(fp,"展開 now(%f,%f,%f)\t%ld\n",now_x,now_y,now_yaw,now_t - start_t);
                    sendMechanism(5);//第一展開
                    status = 3;
                    next_move = 3;
                }else if(wait_num == 0 && fabs(now_x) > 1500){
                    status = 4;
                    next_move = 4;
                }
                break;
            case 4://次の動き選択
                towel_switch = false;//デフォルト状態　trueでタオルを逆にする
                if(sheet_first && fight == 1){
                    if(seats){
                        if(coat == 0){
                            towel_switch = true;//決勝でシーツ先の時はタオルが逆になる
                        }
                        status = 9;//シーツ
                    }else if(towel[0]){
                        status = 5;//タオル１
                        bath = false;
                    }else if(towel[1]){
                        status = 5;//タオル２
                        bath = true;
                    }else{
                        status = 16;//帰る
                    }
                }else{
                    if(towel[0]){
                        status = 5;//タオル１
                        bath = false;
                    }else if(towel[1]){
                        status = 5;//タオル２
                        bath = true;
                    }else if(seats){
                        status = 9;//シーツ
                    }else{
                        status = 16;//帰る
                    }
                }
                break;
            case 5://バスタオルを干す位置に移動する
                if(sheet_first && fight == 1){//タオルを逆にしているときは何もしない。
                    if(wait_num != 0 || spreaded != 1){
                        //fprintf(fp,"シーツ後展開さがり待ち lidar(%d,%d),%d\tG(%d,%d),N(%d,%d,%f)\tRS(%d,%d)\n",lidar_x,lidar_y,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw,(int)rs_data_x,(int)rs_data_y);
                        //break;
                    }
                }else if(fight == 0 && bath == 0){//移動しながら展開
                    //sendSerial(1,2,0);
                    sendMechanism(8);
                }else{
                    //sendSerial(1,2,1);
                    sendMechanism(9);
                }
                stm_auto = false;
                num = 4+2*bath+fight;
                if((bath == 0 && !towel_switch) || (bath == 1 && towel_switch)){
                    setAuto(point[num][coat][0],point[num][coat][1] + 100 + (spreaded == 2)*100,point[num][coat][2]);
                    towel_back = false;
                }else{
                    setAuto(now_x/*point[num][coat][0]*/,NOMAL_Y,0);
                    towel_back = true;//まっすぐ後ろに下がる
                }
                now_t = time(nullptr);
                fprintf(fp,"バスタオル%d補正前 lidar(%d,%d) RS_Y:%d,X:%d\tnow(%f,%f,%f)\t%ld\n",bath+1,lidar_x,lidar_y,(int)rs_data_y,(int)rs_data_x,now_x,now_y,now_yaw,now_t - start_t);
                status = 6;
                next_move = 6;
                rs_goal_y = offset[num-4][coat][1];
                rs_goal_x = offset[num-4][coat][0];
                rs_use = true;//リアルセンス有効化
                count = 0;
                count_towel = 0;
                count_hang = 0;
                towel_x_ok = false;
                towel_y_ok = false;
                break;
            case 6://移動補正
                if(num != 2*bath+fight){
                    num = 2*bath+fight;
                    fprintf(fp,"バスタオル%d移動始め lidar(%d,%d) goal(%d,%d,%f) now(%d,%d,%f)\n",bath+1,lidar_x,lidar_y,(int)goal_x,(int)goal_y,goal_yaw,(int)now_x,(int)now_y,now_yaw);
                }else if(towel_back){
                    if(now_y > (TOWEL_Y + 100)){
                        towel_back = false;
                        goal_x = point[num+4][coat][0];
                        auto_move = true;
                    }else{
                        fprintf(fp,"バスタオル%d下がり待ち lidar(%d,%d),%d\tG(%d,%d),N(%d,%d,%f)\tRS(%d,%d)\n",bath+1,lidar_x,lidar_y,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw,(int)rs_data_x,(int)rs_data_y);
                        break;
                    }
                }

                if(count_towel > 50){
                    skip = true;
                    ROS_INFO("バスタオルタイムアウト");
                    fprintf(fp,"バスタオル%dタイムアウト lidar(%d,%d),RS(%d,%d)\tG(%d,%d),N(%d,%d,%f)\n",bath+1,lidar_x,lidar_y,(int)rs_data_x,(int)rs_data_y,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw);
                    break;
                }else if(towel_x_ok && fabs(now_x - goal_x) < 40 && fabs(now_y - goal_y) < 40 && fabs(now_yaw) < 2){
                    count_towel++;
                }else if(count_towel != 0){
                    count_towel = 0;
                }

                now_t = time(nullptr);
                if(!towel_y_ok && spreaded == 1 && wait_num == 0 && sheet_first && fight && fabs(now_x - point[num+4][coat][0]) < 400){
                    towel_y_ok = true;
                    goal_y = TOWEL_Y + 100;
                    auto_move = true;
                }
                if(!towel_y_ok && fabs(goal_x - now_x) < 300 && fabs(goal_y - now_y) < 300 && bath == 0 && (!sheet_first || !fight )){
                    towel_y_ok = true;
                    auto_move = true;
                }
                if(fabs(now_x - point[num+4][coat][0]) < 400 && !towel_y_ok && bath == 1 && (!sheet_first || !fight)){
                    //next_move = 7;
                    towel_y_ok = true;
                    goal_y = TOWEL_Y + 100;
                    auto_move = true;
                }else if(fabs(goal_x - point[num+4][coat][0]) < 250 && fabs(now_yaw) < 2 && (!towel_x_ok/* || rs_data_x == 0*/)){
                    lidar_x_diff = (lidar_x - offset[num][coat][2]);//フェンス
                    if(fabs(lidar_x_diff) > 200/* || (coat ? fabs(now_x + lidar_x + 100) : fabs(now_x + lidar_x - 4830)) < 300*/){
                        lidar_x_diff = lidar_x - offset[num][coat][3];//台座
                        if(fabs(lidar_x_diff) < 50 && (!towel_switch || fabs(now_x - point[num+4][coat][0]) < 100)){//振動防止
                            goal_x = now_x + lidar_x_diff;
                            lidar_x_ok = true;
                            fprintf(fp,"バスタオル%d補正X 台座 lidar(%d,%d),%d\tG(%d,%d),N(%d,%d,%f)\tRS(%d,%d)\t%ld\n",bath+1,lidar_x,lidar_y,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw,(int)rs_data_x,(int)rs_data_y,now_t - start_t);
                        }else{
                            lidar_x_ok = false;
                            //fprintf(fp,"バスタオル%d補正X 無視 lidar_x:%d,%d\tG(%d,%d),N(%d,%d)\tRS(%d,%d)\t%ld\n",bath+1,lidar_x,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,(int)rs_data_x,(int)rs_data_y,now_t - start_t);
                        }
                    }else{
                        goal_x = now_x + lidar_x_diff;
                        lidar_x_ok = true;
                        fprintf(fp,"バスタオル%d補正X フェンス lidar(%d,%d),%d\tG(%d,%d),N(%d,%d,%f)\tRS(%d,%d)\twait:%d\t%ld\n",bath+1,lidar_x,lidar_y,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw,(int)rs_data_x,(int)rs_data_y,wait_num,now_t - start_t);
                    }
                }else if(lidar_x_ok){
                    lidar_x_ok = false;
                }
                if(!lidar_x_ok){
                    if(fabs(now_x - goal_x) < 300 && fabs(tmp_goal_x - point[num+4][coat][0]) < 250 && rs_data_x != 0){
                        if(rs_data){
                            goal_x = tmp_goal_x;
                            fprintf(fp,"バスタオル%d補正X RS_X:%d,%d Y:%d\tG(%d,%d),N(%d,%d,%f)\tlidar(%d,%d)\t%ld\n",bath+1,(int)rs_data_x,(int)rsx_diff,(int)rs_data_y,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw,lidar_x,lidar_y,now_t - start_t);
                        }
                        if(count != 0){
                            count = 0;
                        }
                    }else{
                        if(count > 25 && !towel_x_ok){
                            goal_x = point[num+4][coat][0];
                        }else{
                            count++;
                        }
                        fprintf(fp,"バスタオル%d補正X 無し lidar(%d,%d),%d\tG(%d,%d),N(%d,%d,%f)\tRS(%d,%d)\t%ld\n",bath+1,lidar_x,lidar_y,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw,(int)rs_data_x,(int)rs_data_y,now_t - start_t);
                    }
                }else if(count != 0){
                    count = 0;
                }

                if(!auto_move){
                    auto_move = true;
                }
                if(wait_num == 0 && count_hang < 15 && bath == 1){
                    count_hang++;
                }
                if(towel_y_ok && (count_hang >= 15 || bath == 0) && fabs(now_x - goal_x) < 10 && fabs(now_v_x) < 30 && fabs(now_yaw) < 1 && !towel_x_ok){
                    towel_x_ok = true;
                    next_move = 7;
                    auto_move = true;
                    goal_y = TOWEL_Y + 30;
                    if(rs_data_x != 0 && fabs(rsx_diff) < 100){
                        rs_goal_x = rs_data_x;
                    }
                    fprintf(fp,"バスタオル%d補正X OK lidar(%d,%d),RS(%d,%d),%d\tG(%d,%d),N(%d,%d,%f)\n",bath+1,lidar_x,lidar_y,(int)rs_data_x,(int)rs_data_y,(int)rsx_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw);
                }else if(towel_x_ok){
                    lidar_y_diff = (lidar_y - offset[0][0][4]);
                    if(fabs(lidar_y_diff) < 150 /*&& fabs(now_y - goal_y) < 200*/ && (now_y - TOWEL_Y) > -250 && fabs(now_yaw) < 2){
                        goal_y = now_y + lidar_y_diff;
                        fprintf(fp,"バスタオル%d補正Y lidar(%d,%d),%d\tG(%d,%d),N(%d,%d,%f)\tRS(%d,%d)\n",bath+1,lidar_x,lidar_y,lidar_y_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw,(int)rs_data_x,(int)rs_data_y);
                    }else if(tmp_goal_y > (TOWEL_Y - 120) && tmp_goal_y < (TOWEL_Y + 120) && /*rsy_diff > -150*/ fabs(now_y - goal_y) < 200 && (now_y - TOWEL_Y) < 100){
                        if(rs_data){
                            goal_y = tmp_goal_y;
                            fprintf(fp, "バスタオル%d補正Y RS_Y:%d,%d X:%d\tG(%d,%d),N(%d,%d,%f)\n", bath + 1, (int)rs_data_y, (int)rsy_diff, (int)rs_data_x, (int)goal_x, (int)goal_y, (int)now_x, (int)now_y,now_yaw);
                        }
                    }else{
                        goal_y = TOWEL_Y;
                        fprintf(fp, "RS無視 RS_Y:%d,%d,%d X:%d\tlidar(%d,%d),%d\tG(%d,%d),N(%d,%d,%f)\n", (int)rs_data_y, (int)rsy_diff, (int)tmp_goal_y, (int)rs_data_x,lidar_x,lidar_y,lidar_y_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw);
                    }
                }else if(rs_data){
                    rs_data = false;
                }
                break;
            case 7://バスタオルかける
                //rs_use = false;//リアルセンス無効化
                if(towel[bath]){
                    //バスタオルノード起動
                    now_t = time(nullptr);
                    fprintf(fp,"バスタオル%d lidar(%d,%d) RS_Y:%d,X:%d\tnow(%f,%f,%f)\t%ld\n",bath+1,lidar_x,lidar_y,(int)rs_data_y,(int)rs_data_x,now_x,now_y,now_yaw,now_t - start_t);
                    if(towel_switch){
                        sendMechanism((!bath)+1);
                    }else{
                        sendMechanism(bath+1);
                    }
                    next_move = 8;
                    status = 0;
                }else{
                    status = 8;
                }
                break;
            case 8://次の動きの選択
                if(towel_switch){
                    if(bath && towel[0]){
                        bath = false;
                        status = 5;//タオル１
                    }else{
                        status = 16;//帰る
                    }
                }else if(bath){
                    if(seats){
                        status = 9;//シーツ
                    }else{
                        status = 16;//帰る
                    }
                }else{
                    if(towel[1]){
                        bath = true;
                        status = 5;//タオル２
                    }else if(seats){
                        status = 9;//シーツ
                    }else{
                        status = 16;//帰る
                    }
                }
                break;
            case 9://第二展開
                //sendMechanism(6);//展開しながら移動
                status = 10;
                next_move = 10;
                break;
            case 10://干し始め位置へ移動
                stm_auto = false;
                setAuto(/*point[2][coat][0] + 50*/now_x,point[2][coat][1],point[2][coat][2]);
                now_t = time(nullptr);
                fprintf(fp,"シーツ補正前 lidar(%d,%d) RS_Y:%d,X:%d\tnow(%f,%f,%f)\t%ld\n",lidar_x,lidar_y,(int)rs_data_y,(int)rs_data_x,now_x,now_y,now_yaw,now_t - start_t);
                status = 11;
                next_move = 11;
                rs_use = true;//リアルセンス有効化
                rs_goal_y = offset[4][coat][1];
                rs_goal_x = offset[4][coat][0] - 50;
                count = 0;
                towel_back = true;
                break;
            case 11://補正動作
                if(towel_back){//一定分まっすぐ後ろに下がる
                    if(now_y > (TOWEL_Y + 100)){
                        goal_x = point[2][coat][0] + 50;
                        towel_back = false;
                        auto_move = true;
                    }else{
                        fprintf(fp,"バスタオル後下がり待ち lidar(%d,%d),%d\tG(%d,%d),N(%d,%d,%f)\tRS(%d,%d)\n",lidar_x,lidar_y,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,now_yaw,(int)rs_data_x,(int)rs_data_y);
                        break;
                    }
                }

                if(count > 50){
                    status = 12;
                    next_move = 12;
                    skip = true;
                    ROS_INFO("シーツタイムアウト");
                    fprintf(fp,"シーツタイムアウト lidar(%d,%d) RS_Y:%d,X:%d\tnow(%d,%d,%f)\n",lidar_x,lidar_y,(int)rs_data_y,(int)rs_data_x,(int)now_x,(int)now_y,now_yaw);
                    break;
                }else if(fabs(now_x - goal_x) < 50 && fabs(now_y - goal_y) < 50 && spreaded == 2){
                    count++;
                }else if(count > 0){
                    count = 0;
                }
                lidar_x_diff = lidar_x - offset[4][coat][2] + ((next_move == 12 && fabs(now_y - goal_y) < 100) ? 0 : 50);//フェンス
                if(fabs(now_y - goal_y) < 100 && next_move == 12){
                    rs_goal_x = offset[4][coat][0];
                }
                if(limit_left){
                    if((rs_data_y == 0 || fabs(rsy_diff) < 20) && fabs(goal_y - now_y) < 20 && spreaded == 2){
                        status = 12;
                        next_move = 12;
                        skip = true;
                    }else if(fabs(goal_y - now_y) < 120){
                        goal_x = now_x;
                    }else{
                        goal_x = now_x + 15;
                    }
                    fprintf(fp,"シーツリミット左 lidar(%d,%d) RS_Y:%d,X:%d\tnow(%d,%d,%f)\n",lidar_x,lidar_y,(int)rs_data_y,(int)rs_data_x,(int)now_x,(int)now_y,now_yaw);
                }else if(fabs(rsx_diff) < 300 && fabs(tmp_goal_x - point[2][coat][0]) < 300 && spreaded == 2){
                    //goal_x = now_x - 50;
                    if(rs_data){
                        goal_x = tmp_goal_x;
                        fprintf(fp,"シーツ補正X RS_X:%d,%d Y:%d\tG(%d,%d),N(%d,%d)\tlidar(%d,%d)\n",(int)rs_data_x,(int)rsx_diff,(int)rs_data_y,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,lidar_x,lidar_y);
                    }
                }else if(fabs(lidar_x_diff) > 300 || fabs(goal_x - now_x) > 300 || fabs(goal_x - point[2][coat][0]) > 300 || fabs(now_yaw) > 1){
                    goal_x = point[2][coat][0] + ((next_move == 12 && fabs(now_y - goal_y) < 100) ? 0 : 50);
                    fprintf(fp,"シーツ補正X 無視lidar(%d,%d),%d\tG(%d,%d),N(%d,%d)\tRS(%d,%d)\n",lidar_x,lidar_y,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,(int)rs_data_x,(int)rs_data_y);
                }else{
                    goal_x = now_x + lidar_x_diff;
                    fprintf(fp,"シーツ補正X lidar(%d,%d),%d\tG(%d,%d),N(%d,%d)\tRS(%d,%d)\n",lidar_x,lidar_y,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,(int)rs_data_x,(int)rs_data_y);
                }
                if(spreaded != 2 && wait_num == 0 && fabs(now_y - goal_y) < 50 && fabs(now_x - goal_x) < 50){
                    sendMechanism(6);//ある程度の場所に行ってから展開
                    now_t = time(nullptr);
                    fprintf(fp,"展開２ lidar(%d,%d) now(%f,%f,%f)\tRS(%d,%d)\t%ld\n",lidar_x,lidar_y,now_x,now_y,now_yaw,(int)rs_data_x,(int)rs_data_y,now_t - start_t);
                    if(rs_data){
                        rs_data = false;
                    }
                }else if(wait_num == 0 && spreaded == 2 && (fabs(now_x - goal_x) < 50 || (SEATS_Y - now_y) < 200)){
                    lidar_y_diff = lidar_y - offset[4][0][4];
                    if(now_y > (SEATS_Y + 100)){
                        goal_y = SEATS_Y;
                        fprintf(fp,"シーツ補正Y 無し RS(%d,%d) G(%d,%d),N(%d,%d)\tlidar(%d,%d)\n",(int)rs_data_x,(int)rs_data_y,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,lidar_x,lidar_y);
                    }else if(fabs(SEATS_Y  - tmp_goal_y) < 300 && tmp_goal_y < (SEATS_Y + 150)&& rsy_diff < 120 && (now_y - SEATS_Y) < 200){
                        if(rs_data){
                            goal_y = tmp_goal_y;
                            fprintf(fp,"シーツ補正Y RS_Y:%d,%d X:%d\tG(%d,%d),N(%d,%d)\n",(int)rs_data_y,(int)rsy_diff,(int)rs_data_x,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y);
                        }
                    }else if(fabs(lidar_y_diff) < 400 && goal_y < (SEATS_Y + 200) && fabs(now_yaw) < 2){
                        goal_y = now_y + lidar_y_diff;
                        fprintf(fp,"シーツ補正Y lidar(%d,%d),%d\tG(%d,%d),N(%d,%d)\tRS(%d,%d)\n",lidar_x,lidar_y,lidar_y_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,(int)rs_data_x,(int)rs_data_y);
                    }else{
                        goal_y = SEATS_Y;
                        fprintf(fp,"シーツ　RS無視 RS_Y:%d,%d,%d X:%d\tlidar_y:%d,%d\n",(int)rs_data_y,(int)rsy_diff,(int)tmp_goal_y,(int)rs_data_x,lidar_y,lidar_y_diff);
                    }
                    if(goal_y == (SEATS_Y - 300)){
                        goal_y = SEATS_Y;
                    }
                    next_move = 12;
                    auto_move = true;
                }else if(!auto_move){
                    auto_move = true;
                }
                break;
            case 12://干し準備
                //干し準備ノード起動
                now_t = time(nullptr);
                fprintf(fp,"シーツはじめ lidar(%d,%d) now(%f,%f,%f)\tRS(%d,%d)\t%ld\n",lidar_x,lidar_y,now_x,now_y,now_yaw,(int)rs_data_x,(int)rs_data_y,now_t - start_t);
                sendMechanism(3);
                status = 0;
                next_move = 13;
                break;
            case 13://干し終わり位置まで移動
                stm_auto = false;
                setAuto(point[3][coat][0]/*now_x*/,now_y/* -100 *//*point[3][coat][1]*/,point[3][coat][2]);
                status = 14;
                next_move = 14;
                rs_use = true;//リアルセンス有効化（一応）
                rs_goal_y = offset[5][coat][1];
                rs_goal_x = offset[5][coat][0];
                count = 0;
                break;
            case 14://補正動作
                if(count < 70 && fabs(now_x - goal_x) > 500){
                    count++;
                    fprintf(fp,"干し右よけ待ち RS_X:%d,%d\tG(%d,%d),N(%d,%d)\tlidar(%d,%d)\n",(int)rs_data_x,(int)rsx_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,lidar_x,lidar_y);
                    break;
                }else if(next_move == 14){
                    goal_x = point[3][coat][0];
                    auto_move = true;
                    next_move = 15;
                }

                if(limit_right){
                    goal_x = now_x - 10;
                    skip = true;
                    status = 15;
                    fprintf(fp,"シーツリミット右 lidar(%d,%d)\tnow(%d,%d,%f)\n",lidar_x,lidar_y,(int)now_x,(int)now_y,now_yaw);
                }else{
                    lidar_x_diff = lidar_x - offset[5][coat][2];//フェンス
                    if(fabs(now_x - goal_x) < 200){
                        goal_x = now_x + 50;
                        fprintf(fp,"干しリミットまで RS_X:%d,%d\tG(%d,%d),N(%d,%d)\tlidar(%d,%d)\n",(int)rs_data_x,(int)rsx_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y,lidar_x,lidar_y);
                    }else if(fabs(now_x - point[3][coat][0]) < 400 && fabs(rsx_diff) < 300 && rs_data_x != 0){
                        if(rs_data){
                            goal_x = tmp_goal_x;
                            fprintf(fp,"干し補正X RS_X:%d,%d\tG(%d,%d),N(%d,%d)\n",(int)rs_data_x,(int)rsx_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y);
                        }
                    }else if(fabs(lidar_x_diff) > 300 || fabs(now_x - point[3][coat][0] > 350) || fabs(now_yaw) > 2){
                        goal_x = point[3][coat][0];
                        fprintf(fp,"干し補正X 無視lidar(%d,%d),%d\tG(%d,%d),N(%d,%d)\n",lidar_x,lidar_y,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y);
                    }else{
                        goal_x = now_x + lidar_x_diff;
                        fprintf(fp,"干し補正X lidar(%d,%d),%d\tG(%d,%d),N(%d,%d)\n",lidar_x,lidar_y,lidar_x_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y);
                    }
                }
                lidar_y_diff = lidar_y - offset[5][0][4];
                if(fabs(rsy_diff) < 100 && tmp_goal_y < (SEATS_Y + 200) && rs_data_y != 0){
                    if(rs_data){
                        rs_data = false;
                        goal_y = tmp_goal_y;
                        fprintf(fp,"干し補正Y RS_Y:%d,%d\tG(%d,%d),N(%d,%d)\n",(int)rs_data_y,(int)rsy_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y);
                    }
                }else if(fabs(lidar_y_diff) < 400 && (now_x + lidar_y_diff) < (SEATS_Y + 50) && fabs(now_yaw) < 2){
                    goal_y = now_y + lidar_y_diff;
                    fprintf(fp,"干し補正Y lidar(%d,%d),%d\tG(%d,%d),N(%d,%d)\n",lidar_x,lidar_y,lidar_y_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y);
                }else{
                    rs_data = false;
                    goal_y = SEATS_Y;
                    fprintf(fp,"干し補正Y 無視 lidar(%d,%d),%d\tG(%d,%d),N(%d,%d)\n",lidar_x,lidar_y,lidar_y_diff,(int)goal_x,(int)goal_y,(int)now_x,(int)now_y);
                }
                break;
            case 15://干し終わり動作
                rs_use = false;//リアルセンス無効化
                //干し終わりノード起動
                now_t = time(nullptr);
                fprintf(fp,"シーツ終わり lidar(%d,%d) now(%f,%f,%f)\tRS(%d,%d)\t%ld\n",lidar_x,lidar_y,now_x,now_y,now_yaw,(int)rs_data_x,(int)rs_data_y,now_t - start_t);
                sendMechanism(4);
                status = 0;
                next_move = 16;
                break;
            case 16://帰れる位置に移動する
                stm_auto = false;
                cmdnum(5,-1,VMAX[spreaded],AMAX[spreaded]);
                setAuto(now_x - 10,NOMAL_Y + 400,0);
                next_move = 17;
                status = 17;
                break;
            case 17://帰り準備
                if(now_y < NOMAL_Y + 450 && now_y > TOWEL_Y + 200){
                    sendSerial(1,5,-2);//端をつかむソレノイドを開放する
                    if(sheet_first){//シーツを先にした時にタオルを干しに行く
                        if(coat){
                            if(towel[0]){
                                status = 5;//タオル２
                                next_move = 5;
                                bath = false;
                                towel_switch = false;
                                goal_y = NOMAL_Y + 200;
                            }else if(towel[1]){
                                status = 5;//タオル１
                                next_move = 5;
                                bath = true;
                                towel_switch = false;
                                goal_y = NOMAL_Y + 200;
                            }else{
                                status = 18;//帰る
                                next_move = 18;
                            }
                        }else{
                            if(towel[1]){
                                status = 5;//タオル２
                                next_move = 5;
                                bath = true;
                                towel_switch = true;
                                goal_y = NOMAL_Y + 200;
                            }else if(towel[0]){
                                status = 5;//タオル１
                                next_move = 5;
                                bath = false;
                                towel_switch = true;
                                goal_y = NOMAL_Y + 200;
                            }else{
                                status = 18;//帰る
                                next_move = 18;
                            }
                        }
                    }else{
                        next_move = 18;
                        status = 18;
                    }
                    if(spreaded > 0 /*&& false*/){
                        now_t = time(nullptr);
                        fprintf(fp,"帰り準備 now(%f,%f,%f)\t%ld\n",now_x,now_y,now_yaw,now_t - start_t);
                        sendMechanism(7);//圧縮動作
                    }
                }
                break;
            case 18://スタートゾーンへ戻る
                //ここで動作を止める
                //status = 0;
                //next_move = 0;
                //skip = true;
                //fprintf(fp,"動作終了 now(%f,%f,%f)\t%ld\n",now_x,now_y,now_yaw,now_t - start_t);
                //break;

                stm_auto = false;
                //cmdnum(5,-1,VMAX[spreaded],AMAX[spreaded]);
                setAuto(point[8][coat][0],std::max(now_y,NOMAL_Y)/*point[8][coat][1]*/,point[8][coat][2]);
                status = 19;
                next_move = 19;
                break;
            case 19:
                if(fabs(now_x) < 600){
                    stm_auto = false;
                    //cmdnum(5,-1/*coat*/,VMAX[spreaded],AMAX[spreaded]);
                    setAuto((coat ? -200 : 200),200,0);
                    next_move = 21;
                    status = 20;
                }
                break;
            case 20://スタートゾーンに入った時点で止める
                if(fabs(now_v_x) < 20 && fabs(now_v_y) < 20 && fabs(now_x - goal_x) < 200 && fabs(now_y - goal_y) < 200){
                    skip = true;
                }
                break;
            case 21:
                //cmdnum(5,-1,VMAX[spreaded],AMAX[spreaded]);
                sendSerial(1,4,0);
                now_t = time(nullptr);
                fprintf(fp,"ホーム到着 now(%f,%f,%f)\t%ld\n",now_x,now_y,now_yaw,now_t - start_t);
                status = 0;
                next_move = 0;
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    sendSerial(255,255,0);
    ros::spinOnce();
    fclose(fp);

    fp = fopen(filename,"r");//書き込んでいるか調べる
    if(fp == nullptr){
        return 0;
    }
    if(fgets(buf,100,fp) == nullptr){//何も書き込んでない時は削除する
        sprintf(buf,"rm -f %s",filename);
        system(buf);
    }
    fclose(fp);
    return 0;
}

void operate(const std_msgs::Float32MultiArray &data){
    int cmd = data.data[0];
    static bool moving = false;
    static int backup_status = 0;
    static int backup_next = 0;
    if(cmd >= 0){
        if(data.data.size() < 4){
            return;
        }
    }
    ROS_INFO("cmd:%d",cmd);
    switch (cmd){
        case -6://リセット
            towel[0] = true;
            towel[1] = true;
            seats = true;
            status = 0;
            next_move = 0;
            ready = false;
            stopped = false;
            cmd1(-2);
            sendMechanism(0);
            sendSerial(255,255,0);
            break;
        case -5://スタート
            if(stopped){//非常停止の後の復帰
                status = backup_status;
                next_move = backup_next;
                stopped = false;
                stm_auto = false;
                if(moving){
                    auto_move = true;
                }
            }else{
                status = 1;
            }
            ready = false;
            break;
        case -4:
            change_data = true;
            break;
        case -3://レディ
            ready = true;
            stopped = false;
            break;
        case -2:
            if(next_move != 0){
                backup_next = next_move;
                backup_status = status;
                stopped = true;
            }
        case -1:
            next_move = 0;
            status = 0;
            if(auto_move){
                skip = true;
                moving = true;
            }else{
                moving = false;
            }
            ready = false;
            cmd1(cmd);//移動系の停止
            //sendMechanism(0);//機構の停止
            sendSerial(255,255,0);
            break;
        case 2:
            goal_x = data.data[1];
            goal_y = data.data[2];
            goal_yaw = data.data[3];
            auto_move = true;
            if(status != 0){
                next_move = status;
            }
            status = 0;
            stopped = false;
            break;
        case 10:
            status = (int)data.data[1];
            next_move = (int)data.data[2];
            spreaded = (bool)data.data[3];
            change_data = true;
            stopped = false;
            break;
        case 11:
            towel[0] = (bool)data.data[1];
            towel[1] = (bool)data.data[2];
            seats = (bool)data.data[3];
            change_data= true;
            stopped = false;
            break;
        case 12:
            coat = (bool)data.data[1];
            fight = (bool)data.data[2];
            change_data = true;
            stopped = false;
            break;
        default:
            motor.publish(data);
            break;
    }
}

inline void autoMove(){
    constexpr double A_MAX_LOOP[3] = {AMAX[0] / LOOP_RATE,AMAX[1] / LOOP_RATE,AMAX[2] / LOOP_RATE};
    constexpr double JARK_LOOP[3] = {JARK[0] / LOOP_RATE,JARK[1] / LOOP_RATE,JARK[2] / LOOP_RATE};
    constexpr double Kp  = 2.5; //自動移動//2.8
    constexpr double Ki[2]  = {2.5,4.5};//0.0004//2.5 4.5
    constexpr double Kd  = 0.01;//0.0006
    static double xKi = Ki[0],yKi = Ki[0];
    static double arc_diff,arc_max;
    static double send_v_x,send_v_y,send_omega;
    static double pid_v_x,pid_v_y,pid_omega;
    static double diff_x,diff_y,diff_yaw;
    static double diff_v_x,diff_v_y,diff_omega;
    static double errer_x,errer_y,errer_omega;
    static double use_v_max = 0;
    static double use_a_x,use_a_y;
    if(use_v_max != (fast_mode ? FAST_V : VMAX[spreaded])){
        use_v_max = (fast_mode ? FAST_V : VMAX[spreaded]);
        arc_diff = M_PI / use_v_max;
        arc_max = 2.6 / M_PI * use_v_max;// 1300 / 500
        ROS_INFO("VMAX:%d",(int)use_v_max);
    }
    if(move_data.data.size() != 4){
        move_data.data.resize(4);
    }
    if(move_data.data[0] != 0){
        move_data.data[0] = 0;
    }
    diff_x = goal_x - now_x;
    diff_y = goal_y - now_y;
    diff_yaw = goal_yaw - now_yaw;
    if(fabs(now_v_x) < 0.05 && fabs(diff_x) < 50 && xKi == Ki[0]){
        xKi = Ki[1];
        errer_x = 0;
        std::cout << "xKi 1" << std::endl;
    }else if(fabs(now_v_x) > 200 && xKi == Ki[1]){
        xKi = Ki[0];
        errer_x = 0;
        std::cout << "xKi 0" << std::endl;
    }
    if(fabs(now_v_y) < 0.05 && fabs(diff_y) < 50 && yKi == Ki[0]){
        yKi = Ki[1];
        errer_y = 0;
        std::cout << "yKi 1" << std::endl;
    }else if(fabs(now_v_y) > 200 && yKi == Ki[1]){
        yKi = Ki[0];
        errer_y = 0;
        std::cout << "yKi 0" << std::endl;
    }

    pid_v_x = constrain(arc_max * atan(arc_diff * diff_x) + errer_x * xKi - diff_v_x * Kd,-use_v_max,use_v_max);
    //pid_v_x = constrain(diff_x * Kp + errer_x * xKi - diff_v_x * Kd,-use_v_max,use_v_max);
    if(fabs(pid_v_x) >= fabs(send_v_x)){
        use_a_x = std::min(use_a_x + JARK_LOOP[spreaded],A_MAX_LOOP[spreaded]);
        if(fabs(send_v_x - pid_v_x) > use_a_x){
            send_v_x += (pid_v_x >= 0 ? 1 : -1)*use_a_x;
        }else{
            send_v_x = pid_v_x;
        }
        errer_x = 0;
        diff_v_x = 0;
    }else{
        if(fabs(diff_x) < 240){
            errer_x += diff_x / LOOP_RATE;
            diff_v_x = (pid_v_x - send_v_x) * LOOP_RATE;
        }else{
            errer_x = 0;
            diff_v_x = 0;
        }
        use_a_x = 0;
        send_v_x = pid_v_x;
    }/*
    if(fabs(diff_x) < 180){
        errer_x += diff_x / LOOP_RATE;
    }else{
        errer_x = 0;
    }*/

    pid_v_y = constrain(arc_max * atan(arc_diff * diff_y) + errer_y * yKi - diff_v_y * Kd,-use_v_max,use_v_max);
    //pid_v_y = constrain(diff_y * Kp + errer_y * yKi - diff_v_y * Kd,-use_v_max,use_v_max);
    if(fabs(pid_v_y) >= fabs(send_v_y)){
        use_a_y = std::min(use_a_y + JARK_LOOP[spreaded],A_MAX_LOOP[spreaded]);
        if(fabs(send_v_y - pid_v_y) > use_a_y){
            send_v_y += (pid_v_y >= 0 ? 1 : -1)*use_a_y;
        }else{
            send_v_y = pid_v_y;
        }
        errer_y = 0;
        diff_v_y = 0;
    }else{
        if(fabs(diff_y) < 240){
            errer_y += diff_y / LOOP_RATE;
            diff_v_y = (pid_v_y - send_v_y) * LOOP_RATE;
        }else{
            errer_y = 0;
            diff_v_y = 0;
        }
        use_a_y = 0;
        send_v_y = pid_v_y;
    }/*
    if(fabs(diff_y) < 180){
        errer_y += diff_y / LOOP_RATE;
    }else{
        errer_y = 0;
    }*/

    pid_omega = constrain(-diff_yaw / 40 + errer_omega / 60 - diff_omega / 100,-0.9,0.9);
    if(fabs(pid_omega) >= fabs(send_omega)){
        if(fabs(send_omega - pid_omega) > 0.05){
            send_omega += (pid_omega >= 0 ? 1 : -1)*0.05;
        }else{
            send_omega = pid_omega;
        }
    }else{
        diff_omega = (pid_omega - send_omega) * LOOP_RATE;
        send_omega = pid_omega;
        errer_omega += -diff_yaw / LOOP_RATE;
    }

    if((fabs(diff_x) <  10 && fabs(diff_y) < 10 && fabs(diff_yaw) < 1
                && fabs(now_v_x) < 15 && fabs(now_v_y) < 15 && fabs(now_omega) < 0.02) || skip){
        send_v_x = 0;
        send_v_y = 0;
        send_omega = 0;
        errer_x = 0;
        errer_y = 0;
        errer_omega = 0;
        diff_v_x = 0;
        diff_v_y = 0;
        diff_omega = 0;
        if(wait_num == 0){
            status = next_move;
        }
        skip = false;
        auto_move = false;
        xKi = Ki[0];
        yKi = Ki[0];
        ROS_INFO("autoMove Fin");
    }
    move_data.data[1] = send_v_x;
    move_data.data[2] = send_v_y;
    move_data.data[3] = send_omega;
    motor.publish(move_data);
}

inline void setAuto(double x,double y,double yaw){
    goal_x = x;
    goal_y = y;
    goal_yaw = yaw;
    if(stm_auto){
        if(move_data.data.size() != 4){
            move_data.data.resize(4);
        }
        if(move_data.data[0] != 1){
            move_data.data[0] = 1;
        }
        move_data.data[1] = x;
        move_data.data[2] = y;
        move_data.data[3] = yaw;
        motor.publish(move_data);
        ros::spinOnce();
    }
    auto_move = true;
}

inline void sendMechanism(int cmd){
    static std_msgs::Int32 data;
    wait_num = cmd;
    data.data = cmd;
    mechanism.publish(data);
    ros::spinOnce();
}

inline void sendSerial(uint8_t id,uint8_t cmd,int16_t data){
    static std_msgs::Int32 send_data;
    send_data.data = (id << 24) + (cmd << 16) + (data & 0xffff);
    mdd.publish(send_data);
    ros::spinOnce();//絶対送信するマン
}

inline void cmdnum(double cmd,double x,double y,double omega){
    if(move_data.data.size() != 4){
        move_data.data.resize(4);
    }
    if(move_data.data[0] != cmd){
        move_data.data[0] = cmd;
    }
    move_data.data[1] = x;
    move_data.data[2] = y;
    move_data.data[3] = omega;
    motor.publish(move_data);
    ros::spinOnce();
}

inline void cmd1(int data){
    if(move_data.data.size() != 1){
        move_data.data.resize(1);
    }
    if(move_data.data[0] != data){
        move_data.data[0] = data;
    }
    motor.publish(move_data);
    ros::spinOnce();
}
