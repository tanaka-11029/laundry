#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <thread>
#include <gtk/gtk.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <RasPiDS3.hpp>
#include "raspi_laundry/PrintStatus.h"

#define DS3 false

constexpr char status_msg[][100] = {
    {"<span foreground='orange' size='160000' weight='ultrabold'>停止中</span>"},
    {"<span foreground='red' size='160000' weight='ultrabold'>非常停止</span>"},
    {"<span foreground='brown' size='160000' weight='ultrabold'>エラー</span>"},
    {"<span foreground='green' size='115000' weight='ultrabold'>シーツへ移動</span>"},
    {"<span foreground='green' size='115000' weight='ultrabold'>タオルへ移動</span>"},
    {"<span foreground='purple' size='115000' weight='ultrabold'>シーツ干し中</span>"},
    {"<span foreground='purple' size='115000' weight='ultrabold'>タオル干し中</span>"},
    {"<span foreground='green' size='115000' weight='ultrabold'>竿の間に移動</span>"},
    {"<span foreground='green' size='115000' weight='ultrabold'>ホームへ移動</span>"},
    {"<span foreground='red' size='160000' weight='ultrabold'>警告</span>"},
    {"<span foreground='green' size='160000' weight='ultrabold'>展開動作</span>"}
};

constexpr char coat_msg[2][100] = {
    {"<span foreground='red' size='100000' weight='ultrabold'>赤コート</span>"},
    {"<span foreground='blue' size='100000' weight='ultrabold'>青コート</span>"}
};

constexpr char fight_msg[2][100] = {
    {"<span foreground='violet' size='100000' weight='ultrabold'>予選</span>"},
    {"<span foreground='gold' size='100000' weight='ultrabold'>決勝</span>"}
};

constexpr char tips_msg[][200] = {
    {"<span foreground='black' size='70000' weight='ultrabold'>動作を設定して\nBキーを押して\nください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>動作を完了するまで\nお待ちください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>全体動作待機中\nです。スペースキーを\n押してください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>停止しました。\n再度動作を設定\nしてください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>STM32からの信号を\n受信できません。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>機構のみ動作待機中\nです。スペースキーを\n押してください。</span>"},
};

constexpr char move_msg[7][2][100] = {
    {{"<span foreground='black' size='30000' weight='bold'>タオル１</span>"},
    {"<span foreground='orange' size='30000' weight='bold'>タオル１</span>"}},
    {{"<span foreground='black' size='30000' weight='bold'>タオル２</span>"},
    {"<span foreground='orange' size='30000' weight='bold'>タオル２</span>"}},
    {{"<span foreground='black' size='30000' weight='bold'>シーツ</span>"},
    {"<span foreground='orange' size='30000' weight='bold'>シーツ</span>"}},
    {{"<span foreground='black' size='30000' weight='bold'>ロック</span>"},
    {"<span foreground='orange' size='30000' weight='bold'>ロック</span>"}},
    {{"<span foreground='black' size='30000' weight='bold'>手動</span>"},
    {"<span foreground='orange' size='30000' weight='bold'>手動</span>"}},
    {{"<span foreground='black' size='30000' weight='bold'>シーツ\n始め</span>"},
    {"<span foreground='orange' size='30000' weight='bold'>シーツ\n始め</span>"}},
    {{"<span foreground='black' size='30000' weight='bold'>シーツ\n終わり</span>"},
    {"<span foreground='orange' size='30000' weight='bold'>シーツ\n終わり</span>"}}
};

constexpr char bool_name[2][10] = {
    {"false"},
    {"true"}
};

constexpr int LOOP_RATE  = 25;
constexpr double AMAX[2] = {300,50}; // mm/s/s
constexpr double VMAX[2] = {1500,250}; // mm/s

GtkWidget* window;//Windowを作る
GtkWidget* input;

GtkWidget* status_num;//Window上で描画する文字
GtkWidget* STATUS;
GtkWidget* COAT;
GtkWidget* tips;
GtkWidget* move_mode[6];

GtkWidget* Gcmd;//Input上で入力を受け付けるエントリー
GtkWidget* Gx;
GtkWidget* Gy;
GtkWidget* Gyaw;

GtkWidget* sheet_on;
GtkWidget* sheet_off;//シーツ機構ごと
GtkWidget* start;
GtkWidget* hbox_sheet;
GtkWidget* hbox_move;

GtkWidget* data_text[25];//input上でデータを表示する

std_msgs::Float32MultiArray move_data;
ros::Publisher *move_pub;
ros::Publisher mdd;
char label_name[200];//sprintfのための変数
bool spreaded = false;
bool coat = false;//0:赤　1:青
bool fight = false;//0:予選 1:決勝
bool skip = false;
bool auto_move = false;
bool warn = false;
bool towel[2] = {true,true};
bool seats[2] = {true,true};
bool stm_auto = false;
bool lock = false;
bool ready = false;
bool manual_move = false;
bool manual_sheet = false;
bool only_move = false;
bool spread_move[2] = {false,false};
bool loop_ok = true;
int stm_status = 0;
int status = 0;
int next_move = 0;
int warn_count = 0;
double manual_v_x = 0,manual_v_y = 0,manual_omega = 0;
double goal_x,goal_y,goal_yaw;
double now_v_x,now_v_y,now_omega;
double now_x,now_y,now_yaw,theta;
#if DS3
RPDS3::DualShock3 controller;
#endif

inline double constrain(double x,double a,double b){
        return (x < a ? a : x > b ? b : x );
}

void button_click(GtkWidget* widget,gpointer data);
inline void autoMove();
inline void sendSerial(uint8_t id,uint8_t cmd,int16_t data,ros::Publisher &pub);
inline void cmdnum(double cmd,double x,double y,double omega);
inline void cmd1(int data);
inline void set_auto(double x,double y,double yaw);
inline void spreadMove();
inline void lowerMove();
inline void hangTowel(bool bath);
inline void sheetStart();
inline void sheetEnd();

void getData(const std_msgs::Float32MultiArray &place){
    static int i,j;
    now_x = place.data[0];
    now_y = place.data[1];
    theta = place.data[2];
    now_yaw = place.data[3];
    now_v_x = place.data[4];
    now_v_y = place.data[5];
    now_omega = place.data[6];
    stm_status = (int)place.data[7];
    if(j > 20){
        j = 0;
        for(i = 0;i < 8;i++){
            sprintf(label_name,"%.4f",place.data[i]);
            gtk_label_set_text(GTK_LABEL(data_text[i]),label_name);
        }
    }else{
        j++;
    }
    if(warn_count != 0){
        warn_count = 0;
    }
}

static void ros_main(int argc,char **argv){
    constexpr double NOMAL_Y = 5900;
    constexpr double SEATS_Y = 6300;
    constexpr double TOWEL_Y = 5500;
    constexpr double point[8][2][3] = {//赤　青
        {{0,NOMAL_Y,0},{0,NOMAL_Y,0}},//ポール間に入る前
        {{1800,NOMAL_Y,0},{-1800,NOMAL_Y,0}},//ポール間に入る
        {{1800,SEATS_Y,0},{-1800,SEATS_Y,0}},//シーツかけ始め
        {{3700,SEATS_Y,0},{-3700,SEATS_Y,0}},//シーツかけ終わり
        {{2750,TOWEL_Y,0},{-2750,TOWEL_Y,0}},//タオル１予選４2090
        {{2090,TOWEL_Y,0},{-2090,TOWEL_Y,0}},//タオル１決勝 2180
        {{3550,TOWEL_Y,0},{-3550,TOWEL_Y,0}},//タオル２予選６3300
        {{3410,TOWEL_Y,0},{-3410,TOWEL_Y,0}}//タオル２決勝 3500
    };
    ros::init(argc, argv, "laundry_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    ros::Subscriber place = nh.subscribe("place",100,getData);
    ros::Publisher moter = nh.advertise<std_msgs::Float32MultiArray>("motor",100);
    mdd = nh.advertise<std_msgs::Int32>("Motor_Serial",100);
    move_pub = &moter;
#if DS3
    controller.yReverseSet(true);
#endif
    int count = 0;
    int num = 0;
    int last_status,last_next;
    double speed = 2;
    bool zero = false;
    bool moter4 = false;
    bool last_towel[2];
    bool last_seats[2];
    bool bath = false;
    ROS_INFO("Laundry Node start");
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
                sendSerial(15,255,0,mdd);
            }
            if(controller.press(RPDS3::UP)){
                sendSerial(15,4,100,mdd);
                moter4 = true;
            }else if(controller.press(RPDS3::DOWN)){
                sendSerial(15,4,-100,mdd);
                moter4 = true;
            }else if(controller.release(RPDS3::UP) || controller.release(RPDS3::DOWN)){
                sendSerial(15,4,0,mdd);
                if(moter4){
                    sendSerial(15,4,0,mdd);
                    moter4 = false;
                }else{
                    sendSerial(15,2,0,mdd);
                }
            }
        }else{
            if(controller.press(RPDS3::SELECT)){
                cmd1(-2);
                sendSerial(15,255,0,mdd);
            }
            if(controller.press(RPDS3::UP)){
                sendSerial(15,2,100,mdd);
            }else if(controller.press(RPDS3::DOWN)){
                sendSerial(15,2,-100,mdd);
            }else if(controller.release(RPDS3::UP) || controller.release(RPDS3::DOWN)){
                if(moter4){
                    sendSerial(15,4,0,mdd);
                    moter4 = false;
                }else{
                    sendSerial(15,2,0,mdd);
                }
            }
        }

        if(controller.press(RPDS3::RIGHT) && !controller.button(RPDS3::START)){
            sendSerial(15,3,100,mdd);
        }else if(controller.press(RPDS3::LEFT)){
            sendSerial(15,3,-100,mdd);
        }else if(controller.release(RPDS3::LEFT) || controller.release(RPDS3::RIGHT)){
            sendSerial(15,3,0,mdd);
        }

        if(controller.press(RPDS3::SQUARE)){//ソレノイド
            sendSerial(1,9,1,mdd);
        }else if(controller.release(RPDS3::SQUARE)){
            sendSerial(1,9,-1,mdd);
        }
        if(controller.press(RPDS3::CIRCLE)){
            sendSerial(15,9,2,mdd);
        }else if(controller.release(RPDS3::CIRCLE)){
            sendSerial(15,9,-2,mdd);
        }
        if(controller.press(RPDS3::TRIANGLE)){
            sendSerial(15,10,1,mdd);
        }else if(controller.release(RPDS3::TRIANGLE)){
            sendSerial(15,10,-1,mdd);
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
        manual_v_x = 2*speed*left_x;
        manual_v_y = 2*speed*left_y;
        manual_omega = speed*(left_t - right_t)/300;
#endif
        if(auto_move){
            if(stm_auto){
                if(stm_status == 0){
                    auto_move = false;
                    status = next_move;
                }
            }else{
                autoMove();
            }
        }else if(manual_move){
            if(manual_v_x != 0 || manual_v_y != 0 || manual_omega != 0){
                if(!zero){
                    zero = true;
                }
                //ROS_INFO("manual:%f,%f,%f",manual_v_x,manual_v_y,manual_omega);
                cmdnum(0,manual_v_x,manual_v_y,manual_omega);
            }else if(zero){
                cmdnum(0,0,0,0);
                zero = false;
            }
        }
        if(warn_count >= LOOP_RATE*30){
            if(!warn){
                ROS_WARN("UNCONNECTED TO STM32");
                button_click(NULL,GINT_TO_POINTER(7));
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[9]);
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[4]);
                warn = true;
            }
        }else{
            warn_count++;
            if(warn){
                ROS_INFO("CONNECTED TO STM32");
                button_click(NULL,GINT_TO_POINTER(7));
                warn = false;
            }
        }
        switch (status) {
            case 0:
                break;
            case 1:
                if(fabs(now_x) >= 1500 && fabs(now_y) > 5200){
                    status = 3;
                    break;
                }
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[7]);
                stm_auto = false;
                cmdnum(5,coat,VMAX[spreaded],AMAX[spreaded]);
                set_auto(point[0][coat][0],point[0][coat][1],point[0][coat][2]);
                status = 2;//pixyを使わない時
                next_move = 2;
                break;
            case 2://ポール間に移動
                if(now_y > 5700){
                    stm_auto = false;
                    cmdnum(5,-1,VMAX[spreaded],AMAX[spreaded]);
                    set_auto(point[1][coat][0],point[1][coat][1],point[1][coat][2]);
                    next_move = 3;
                    status = 0;//pixyを使わない時
                }
                break;
            case 3://展開動作
                if(fabs(now_x) < 1400){
                    break;
                }
                if(spreaded){
                    status = 4;
                }else{
                    spreadMove();
                }
                break;
            case 4://次の動き選択
                if(towel[0]){
                    status = 5;//タオル１
                    bath = false;
                }else if(towel[1]){
                    status = 5;//タオル２
                    bath = true;
                }else if(seats[0]){
                    status = 9;//シーツ始め
                }else{
                    status = 15;//帰る
                }
                break;
            case 5://バスタオルを干す位置に移動する
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[4]);
                stm_auto = false;
                num = 4+2*bath+fight;
                set_auto(point[num][coat][0],point[num][coat][1],point[num][coat][2]);
                status = 6;
                break;
            case 6://移動補正
                status = 0;
                next_move = 7;
                break;
            case 7://バスタオルかける
                if(towel[bath]){
                    hangTowel(bath);
                }else{
                    status = 8;
                }
                break;
            case 8://次の動きの選択
                if(bath){
                    if(seats[0]){
                        status = 9;//シーツ
                    }else{
                        status = 15;//帰る
                    }
                }else{
                    if(towel[1]){
                        bath = true;
                        status = 5;//タオル２
                    }else if(seats[0]){
                        status = 9;//シーツ
                    }else{
                        status = 15;//帰る
                    }
                }
                break;
            case 9:
                //干し始め位置へ移動
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[3]);
                stm_auto = false;
                set_auto(point[2][coat][0],point[2][coat][1],point[2][coat][2]);
                status = 10;
                break;
            case 10://補正動作
                next_move = 11;
                status = 0;
                break;
            case 11:
                //干し準備
                if(seats[0]){
                    sheetStart();
                }else{
                    status = 12;
                }
                break;
            case 12://干し終わり位置まで移動
                stm_auto = false;
                set_auto(point[3][coat][0],point[3][coat][1],point[3][coat][2]);
                status = 13;
                break;
            case 13://補正動作
                next_move = 14;
                status = 0;
                break;
            case 14://干し終わり動作
                if(seats[1]){
                    sheetEnd();
                }else{
                    status = 15;
                }
                break;
            case 15://帰れる位置に移動する
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[7]);
                stm_auto = false;
                cmdnum(5,-1,VMAX[spreaded],AMAX[spreaded]);
                set_auto(point[1][coat][0],point[1][coat][1],point[1][coat][2]);
                next_move = 16;
                status = 0;
                break;
            case 16://帰り準備
                if(spreaded){
                    lowerMove();
                }else{
                    status = 17;
                }
                break;
            case 17://スタートゾーンへ戻る
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[8]);
                stm_auto = false;
                cmdnum(5,-1,VMAX[spreaded],AMAX[spreaded]);
                set_auto(0,NOMAL_Y,0);
                status = 18;
                next_move = 18;
                break;
            case 18:
                if(fabs(now_x) < 400){
                    stm_auto = false;
                    cmdnum(5,coat,VMAX[spreaded],AMAX[spreaded]);
                    set_auto(0,0,0);
                    next_move = 19;
                    status = 0;
                }
                break;
            case 19:
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
                cmdnum(5,-1,VMAX[spreaded],AMAX[spreaded]);
                status = 0;
                next_move = 0;
                break;
        }
        if(last_status != status || last_next != next_move){
            sprintf(label_name,"<span foreground='black' size='70000' weight='ultrabold'>ステータス:%d  次:%d</span>",status,next_move);
            gtk_label_set_markup(GTK_LABEL(status_num),label_name);
            gtk_label_set_text(GTK_LABEL(data_text[11]),bool_name[spreaded]);
            last_status = status;
            last_next = next_move;
            ROS_INFO("status : %d\tnext : %d",status,next_move);
            ROS_INFO("now(%f,%f,%f)",now_x,now_y,now_yaw);
            ROS_INFO("goal(%f,%f,%f)", goal_x,goal_y,goal_yaw);
        }
        if(manual_sheet){
            if(seats[0] != last_seats[0]){
                last_seats[0] = seats[0];
                gtk_label_set_markup(GTK_LABEL(move_mode[2]),move_msg[5][seats[0]]);
            }
            if(seats[1] != last_seats[1]){
                last_seats[1] = seats[1];
                gtk_label_set_markup(GTK_LABEL(move_mode[3]),move_msg[6][seats[1]]);
            }
        }else{
            if(seats[0] != last_seats[0] || seats[1] != last_seats[1]){
                last_seats[0] = seats[0];
                last_seats[1] = seats[1];
                gtk_label_set_markup(GTK_LABEL(move_mode[2]),move_msg[2][seats[0] || seats[1]]);
            }
        }
        if(towel[0] != last_towel[0]){
            last_towel[0] = towel[0];
            gtk_label_set_markup(GTK_LABEL(move_mode[0]),move_msg[0][towel[0]]);
        }
        if(towel[1] != last_towel[1]){
            last_towel[1] = towel[1];
            gtk_label_set_markup(GTK_LABEL(move_mode[1]),move_msg[1][towel[1]]);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    sendSerial(255,255,0,mdd);
    gtk_main_quit();
    exit(0);
}

inline void spreadMove(){
    static int count;
    gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[10]);
    while(ros::ok() && loop_ok){
        if(count < 60){//ダミー動作
            count++;
        }else{
            count = 0;
            spreaded = true;
            break;
        }
        ros::spinOnce();
    }
}

inline void lowerMove(){
    static int count;
    gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[10]);
    while(ros::ok() && loop_ok){
        if(count < 60){//ダミー動作
            count++;
        }else{
            count = 0;
            spreaded = false;
            break;
        }
        ros::spinOnce();
    }
}

inline void hangTowel(bool bath){
    static int count;
    gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[6]);
    while(ros::ok() && loop_ok){
        if(count < 60){//ダミー動作
            count++;
        }else{
            count = 0;
            towel[bath] = false;
            break;
        }
        ros::spinOnce();
    }
}

inline void sheetStart(){
    static int count;
    gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[5]);
    while(ros::ok() && loop_ok){
        if(count < 60){//ダミー動作
            count++;
        }else{
            count = 0;
            seats[0] = false;
            break;
        }
        ros::spinOnce();
    }
}

inline void sheetEnd(){
    static int count;
    gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[5]);
    while(ros::ok() && loop_ok){
        if(count < 60){//ダミー動作
            count++;
        }else{
            count = 0;
            seats[1] = false;
            break;
        }
        ros::spinOnce();
    }
}

void button_click(GtkWidget* widget,gpointer data){
    switch (GPOINTER_TO_INT(data)) {
        case 0://コート切り替え
            if(!lock){
                coat = !coat;
                sprintf(label_name,"%s%s",coat_msg[coat],fight_msg[fight]);
                gtk_label_set_markup(GTK_LABEL(COAT),label_name);
            }
            break;
        case 1://予選・決勝切り替え
            if(!lock){
                fight = !fight;
                sprintf(label_name,"%s%s",coat_msg[coat],fight_msg[fight]);
                gtk_label_set_markup(GTK_LABEL(COAT),label_name);
            }
            break;
        case 2://キャリブレーション
            lock = true;
            cmd1(-1);
            gtk_label_set_markup(GTK_LABEL(move_mode[3+manual_sheet]),move_msg[3][lock]);
            break;
        case 3://タオル１
            if(!ready){
                towel[0] = !towel[0];
            }
            break;
        case 4://タオル２
            if(!ready){
                towel[1] = !towel[1];
            }
            break;
        case 5://シーツ
            if(!ready){
                seats[0] = !seats[0];
                if(!manual_sheet){
                    seats[1] = seats[0];
                }
            }
            break;
        case 6://スタート
            if(manual_sheet){
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[5]);
                manual_move = false;
                gtk_label_set_markup(GTK_LABEL(move_mode[4+manual_sheet]),move_msg[4][manual_move]);
                ready = true;
                only_move = true;
            }else{
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[2]);
                manual_move = false;
                gtk_label_set_markup(GTK_LABEL(move_mode[4+manual_sheet]),move_msg[4][manual_move]);
                ready = true;
                only_move = false;
            }
            break;
        case 7://ストップ
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[3]);
            cmd1(-2);
            if(auto_move){
                skip = true;
            }
            status = 0;
            next_move = 0;
            ready = false;
            break;
        case 8://リセット
            status = 0;
            next_move = 0;
            if(auto_move){
                skip = true;
            }
            ready = false;
            towel[0] = true;
            towel[1] = true;
            seats[0] = true;
            cmd1(-2);
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[0]);
            gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
            break;
        case 9://コマンド実行
            if(ready){
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[1]);
                ready = false;
                if(only_move){
                    loop_ok = true;
                    if(towel[0]){
                        hangTowel(0);
                    }else if(towel[1]){
                        hangTowel(1);
                    }else if(seats[0]){
                        sheetStart();
                    }else if(seats[1]){
                        sheetEnd();
                    }else if(spread_move[0] && !spreaded){
                        spread_move[0] = false;
                        spreadMove();
                    }else if(spread_move[1] && spreaded){
                        spread_move[1] = false;
                        lowerMove();
                    }
                    gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
                    only_move = false;
                }else{
                    status = 1;
                }
            }
            break;
        case 10://ロック解除
            lock = false;
            gtk_label_set_markup(GTK_LABEL(move_mode[3+manual_sheet]),move_msg[3][lock]);
            break;
        case 11://手動操作
            manual_move = !manual_move;
            gtk_label_set_markup(GTK_LABEL(move_mode[4+manual_sheet]),move_msg[4][manual_move]);
            break;
        case 12://シーツ終わり
            if(!ready && manual_sheet){
                seats[1] = !seats[1];
            }
            break;
    }
}

void entryInput(GtkWidget* widget,gpointer data){
    int cmd;
    double x,y,yaw;
    char temp[100];
    const gchar *text;
    text = gtk_entry_get_text(GTK_ENTRY(Gcmd));
    sprintf(temp,"%s",text);
    cmd = atoi(temp);
    if(cmd >= 0){
        text = gtk_entry_get_text(GTK_ENTRY(Gx));
        sprintf(temp,"%s",text);
        x = atof(temp);
        text = gtk_entry_get_text(GTK_ENTRY(Gy));
        sprintf(temp,"%s",text);
        y = atof(temp);
        text = gtk_entry_get_text(GTK_ENTRY(Gyaw));
        sprintf(temp,"%s",text);
        yaw = atof(temp);
        printf("%d : %lf %lf %lf\n",cmd,x,y,yaw);
        switch (cmd) {
            case 10:
                status = (int)x;
                next_move = (int)y;
                spreaded = (bool)yaw;
                break;
            case 11:
                towel[0] = (bool)x;
                towel[1] = (bool)y;
                seats[0] = (bool)yaw;
                break;
            case 12:
                button_click(widget,GINT_TO_POINTER((int)x));
                break;
            case 20:
                sendSerial((int)x,(int)y,(int)yaw,mdd);
                break;
            case 30:
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[(int)x]);
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[(int)y]);
                break;
            default:
                move_data.data.resize(4);
                move_data.data[0] = cmd;
                move_data.data[1] = x;
                move_data.data[2] = y;
                move_data.data[3] = yaw;
                move_pub->publish(move_data);
                break;
        }
    }else{
        if(cmd == -6){
            towel[0] = true;
            towel[1] = true;
            seats[0] = true;
        }else if(cmd == -5){
            status = 1;
        }else if(cmd == -1){
            next_move = 0;
            status = 0;
            if(auto_move){
                skip = true;
            }
            cmd1(-1);
        }else if(cmd == -2){
            if(auto_move){
                skip = true;
            }
            cmd1(-2);
        }else if(cmd == -7 && !manual_sheet){
            manual_sheet = true;
            gtk_button_set_label(GTK_BUTTON(sheet_on),"シーツかけ始め(J)");
            sheet_off = gtk_button_new_with_label("シーツかけ終わり(K)");
            gtk_box_pack_start(GTK_BOX(hbox_sheet),sheet_off,true,true,0);
            gtk_button_set_label(GTK_BUTTON(start),"機構のみスタート(B)");
            g_signal_connect(sheet_off,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(12));
            for(int i = 2;i<5;i++){
                gtk_widget_destroy(move_mode[i]);
                move_mode[i] = gtk_label_new(NULL);
            }
            move_mode[5] = gtk_label_new(NULL);
            gtk_label_set_markup(GTK_LABEL(move_mode[2]),move_msg[5][seats[0]]);
            gtk_label_set_markup(GTK_LABEL(move_mode[3]),move_msg[6][seats[1]]);
            gtk_label_set_markup(GTK_LABEL(move_mode[4]),move_msg[3][lock]);
            gtk_label_set_markup(GTK_LABEL(move_mode[5]),move_msg[4][manual_move]);
            for(int i = 2;i<6;i++){
                gtk_box_pack_start(GTK_BOX(hbox_move),move_mode[i],true,true,0);
            }
            gtk_widget_show_all(window);
        }else if(cmd == -8 && manual_sheet){
            manual_sheet = false;
            gtk_button_set_label(GTK_BUTTON(sheet_on),"シーツ(J)");
            gtk_button_set_label(GTK_BUTTON(start),"スタート(B)");
            gtk_widget_destroy(sheet_off);
            for(int i = 2;i<6;i++){
                gtk_widget_destroy(move_mode[i]);
                move_mode[i] = gtk_label_new(NULL);
            }
            gtk_label_set_markup(GTK_LABEL(move_mode[2]),move_msg[2][seats[0]]);
            gtk_label_set_markup(GTK_LABEL(move_mode[3]),move_msg[3][lock]);
            gtk_label_set_markup(GTK_LABEL(move_mode[4]),move_msg[4][manual_move]);
            for(int i = 2;i<5;i++){
                gtk_box_pack_start(GTK_BOX(hbox_move),move_mode[i],true,true,0);
            }
            gtk_widget_show_all(window);
        }else if(cmd == -9){
            gtk_main_quit();
            exit(0);
        }else{
            move_data.data.resize(1);
            move_pub->publish(move_data);
        }
        x = 0;
        y = 0;
        yaw = 0;
        printf("%d : %lf %lf %lf\n",cmd,x,y,yaw);
    }
}

void key_press(GtkWidget* widget,GdkEventKey *event,gpointer data){
    //printf("key_press:keyval=%d\n",event->keyval);
    switch (event->keyval) {
        case 't'://コート切り替え
            button_click(widget,GINT_TO_POINTER(0));
            break;
        case 'y'://予選・決勝切り替え
            button_click(widget,GINT_TO_POINTER(1));
            break;
        case 'u'://キャリブレーション
            button_click(widget,GINT_TO_POINTER(2));
            break;
        case 'g'://タオル１
            button_click(widget,GINT_TO_POINTER(3));
            break;
        case 'h'://タオル２
            button_click(widget,GINT_TO_POINTER(4));
            break;
        case 'j'://シーツ
            button_click(widget,GINT_TO_POINTER(5));
            break;
        case 'b'://スタート
            button_click(widget,GINT_TO_POINTER(6));
            break;
        case 'n'://ストップ
            button_click(widget,GINT_TO_POINTER(7));
            break;
        case 'm'://リセット
            button_click(widget,GINT_TO_POINTER(8));
            break;
        case ' '://コマンド実行
            button_click(widget,GINT_TO_POINTER(9));
            break;
        case 'p'://ロック解除
            button_click(widget,GINT_TO_POINTER(10));
            break;
        case 'r'://手動操作
            button_click(widget,GINT_TO_POINTER(11));
            break;
        case 'k':
            button_click(widget,GINT_TO_POINTER(12));
            break;
        case 'v':
            button_click(widget,GINT_TO_POINTER(13));
            break;
        case 'w':
            manual_v_y = 300;
            break;
        case 'a':
            manual_v_x = -300;
            break;
        case 's':
            manual_v_y = -300;
            break;
        case 'd':
            manual_v_x = 300;
            break;
        case 'e':
            manual_omega = 0.1;
            break;
        case 'q':
            manual_omega = -0.1;
            break;
    }
}

void key_release(GtkWidget* widget,GdkEventKey *event,gpointer data){
    switch (event->keyval){
        case 'w':
        case 's':
            manual_v_y = 0;
            break;
        case 'a':
        case 'd':
            manual_v_x = 0;
            break;
        case 'e':
        case 'q':
            manual_omega = 0;
            break;
        case 65293:
            entryInput(widget,0);
            break;
    }
}

static void run_main(){
    gtk_main();
}

static void quit_main(GtkWidget *button,gpointer user_data){
    gtk_main_quit();
    exit(0);
}

int main(int argc, char **argv){
    gtk_init(&argc,&argv);

    GtkWidget* coat_mode = gtk_button_new_with_label("コート切り替え(T)");//window上のボタン
    GtkWidget* fight_mode = gtk_button_new_with_label("予選/決勝切り替え(Y)");
    GtkWidget* cali = gtk_button_new_with_label("キャリブレーション/決定(U)");
    GtkWidget* towel1 = gtk_button_new_with_label("タオル１(G)");
    GtkWidget* towel2 = gtk_button_new_with_label("タオル２(H)");
    start = gtk_button_new_with_label("スタート(B)");
    GtkWidget* stop = gtk_button_new_with_label("停止(N)");
    GtkWidget* reset = gtk_button_new_with_label("始めに戻る(M)");
    GtkWidget* manual = gtk_button_new_with_label("手動操作(R)");
    GtkWidget* unlock = gtk_button_new_with_label("ロック解除(P)");
    sheet_on = gtk_button_new_with_label("シーツ(J)");
    sheet_off = gtk_button_new_with_label("シーツかけ終わり(K)");

    GtkWidget* cmd_label = gtk_label_new("CMD");//Input上のラベル
    GtkWidget* x_label = gtk_label_new("X");
    GtkWidget* y_label = gtk_label_new("Y");
    GtkWidget* yaw_label = gtk_label_new("Yaw");

    GtkWidget* send = gtk_button_new_with_label("送る");//input上のボタン
    GtkWidget* run = gtk_button_new_with_label("実行");

    GtkWidget* vbox;//描画補助ボックス
    GtkWidget* hbox;
    GtkWidget* vbigbox;
    GtkWidget* hbigbox;

    GtkWidget* data_flame[25];

    //GtkWidget* grid;//inputで使うグリッド

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);//Windowの描画
    gtk_window_set_position(GTK_WINDOW(window),GTK_WIN_POS_CENTER);
    gtk_window_set_title(GTK_WINDOW(window),"Laundry");
    gtk_container_set_border_width(GTK_CONTAINER(window),10);
    gtk_widget_set_size_request(window,900,1080);
    g_signal_connect(window,"destroy",G_CALLBACK(quit_main),NULL);
    g_signal_connect(G_OBJECT(window),"key-press-event",G_CALLBACK(key_press),NULL);
    g_signal_connect(G_OBJECT(window),"key-release-event",G_CALLBACK(key_release),NULL);

    vbigbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_container_add(GTK_CONTAINER(window),vbigbox);

    status_num = gtk_label_new(NULL);//Window上で描画する文字
    STATUS = gtk_label_new(NULL);
    COAT = gtk_label_new(NULL);
    tips = gtk_label_new(NULL);
    for(int i = 0;i<5;i++){
        move_mode[i] = gtk_label_new(NULL);
        gtk_label_set_markup(GTK_LABEL(move_mode[i]),move_msg[i][i >= 3 ? 0 : 1]);
    }

    sprintf(label_name,"<span foreground='black' size='70000' weight='ultrabold'>ステータス:%d  次:%d</span>",0,0);
    gtk_label_set_markup(GTK_LABEL(status_num),label_name);
    gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
    sprintf(label_name,"%s%s",coat_msg[0],fight_msg[0]);
    gtk_label_set_markup(GTK_LABEL(COAT),label_name);
    gtk_label_set_markup(GTK_LABEL(tips),tips_msg[0]);

    gtk_widget_set_size_request(status_num,900,80);//番号
    gtk_box_pack_start(GTK_BOX(vbigbox),status_num,true,true,0);

    gtk_widget_set_size_request(STATUS,900,250);//ステータス
    gtk_box_pack_start(GTK_BOX(vbigbox),STATUS,true,true,0);

    hbox_move = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,5);
    gtk_widget_set_size_request(hbox_move,900,100);
    for(int i = 0;i<5;i++){
        gtk_box_pack_start(GTK_BOX(hbox_move),move_mode[i],true,true,0);
    }
    gtk_box_pack_start(GTK_BOX(vbigbox),hbox_move,true,true,0);

    hbigbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,5);//ボタン配置
    gtk_widget_set_size_request(hbigbox,900,200);
    gtk_box_pack_start(GTK_BOX(vbigbox),hbigbox,true,true,0);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_widget_set_size_request(vbox,300,200);
    gtk_box_pack_start(GTK_BOX(vbox),coat_mode,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),towel1,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),start,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbigbox),vbox,true,true,0);


    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_widget_set_size_request(vbox,300,200);
    gtk_box_pack_start(GTK_BOX(vbox),fight_mode,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),towel2,true,true,0);

    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,5);
    gtk_box_pack_start(GTK_BOX(hbox),stop,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),unlock,true,true,0);

    gtk_box_pack_start(GTK_BOX(vbox),hbox,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbigbox),vbox,true,true,0);


    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_widget_set_size_request(vbox,300,200);
    gtk_box_pack_start(GTK_BOX(vbox),cali,true,true,0);

    hbox_sheet = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,5);
    gtk_box_pack_start(GTK_BOX(vbox),hbox_sheet,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox_sheet),sheet_on,true,true,0);

    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,5);
    gtk_box_pack_start(GTK_BOX(hbox),manual,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),reset,true,true,0);

    gtk_box_pack_start(GTK_BOX(vbox),hbox,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbigbox),vbox,true,true,0);

    gtk_widget_set_size_request(COAT,900,150);//コート
    gtk_box_pack_start(GTK_BOX(vbigbox),COAT,true,true,0);
    gtk_widget_set_size_request(tips,900,300);//説明
    gtk_box_pack_start(GTK_BOX(vbigbox),tips,true,true,0);

    g_signal_connect(coat_mode,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(0));
    g_signal_connect(fight_mode,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(1));
    g_signal_connect(cali,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(2));
    g_signal_connect(towel1,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(3));
    g_signal_connect(towel2,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(4));
    g_signal_connect(sheet_on,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(5));
    g_signal_connect(start,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(6));
    g_signal_connect(stop,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(7));
    g_signal_connect(reset,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(8));
    g_signal_connect(unlock,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(10));
    g_signal_connect(manual,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(11));
    g_signal_connect(sheet_off,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(12));

    input = gtk_window_new(GTK_WINDOW_TOPLEVEL);//inputの描画
    gtk_window_set_position(GTK_WINDOW(input),GTK_WIN_POS_CENTER);
    gtk_window_set_title(GTK_WINDOW(input),"Operator");
    gtk_container_set_border_width(GTK_CONTAINER(input),10);
    gtk_widget_set_size_request(input,800,200);
    g_signal_connect(input,"destroy",G_CALLBACK(quit_main),NULL);
    g_signal_connect(G_OBJECT(input),"key-release-event",G_CALLBACK(key_release),NULL);

    hbigbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,10);//全体のボックス
    gtk_container_add(GTK_CONTAINER(input),hbigbox);

    Gcmd = gtk_entry_new();//Input上で入力を受け付けるエントリー
    Gx = gtk_entry_new();
    Gy = gtk_entry_new();
    Gyaw = gtk_entry_new();

    vbigbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,2);
    gtk_widget_set_size_request(vbigbox,150,200);
    gtk_box_pack_start(GTK_BOX(hbigbox),vbigbox,false,false,0);

    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,0);
    gtk_widget_set_size_request(hbox,150,150);
    gtk_box_pack_start(GTK_BOX(vbigbox),hbox,true,true,0);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,2);
    gtk_widget_set_size_request(vbox,50,150);
    gtk_box_pack_start(GTK_BOX(vbox),cmd_label,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),x_label,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),y_label,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),yaw_label,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),vbox,true,true,0);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,2);
    gtk_widget_set_size_request(vbox,100,150);
    gtk_box_pack_start(GTK_BOX(vbox),Gcmd,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),Gx,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),Gy,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),Gyaw,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),vbox,true,true,0);

    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,5);
    gtk_widget_set_size_request(hbox,150,50);
    gtk_box_pack_start(GTK_BOX(hbox),send,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),run,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbigbox),hbox,true,true,0);

    data_flame[0] = gtk_frame_new("now_x");
    data_flame[1] = gtk_frame_new("now_y");
    data_flame[2] = gtk_frame_new("theta");
    data_flame[3] = gtk_frame_new("now_yaw");
    data_flame[4] = gtk_frame_new("now_v_x");
    data_flame[5] = gtk_frame_new("now_v_y");
    data_flame[6] = gtk_frame_new("now_omega");
    data_flame[7] = gtk_frame_new("stm_status");
    data_flame[8] = gtk_frame_new("pixy_start");
    data_flame[9] = gtk_frame_new("pixy_end");
    data_flame[10] = gtk_frame_new("pixy_intersection");
    data_flame[11] = gtk_frame_new("spreaded");
    data_flame[12] = gtk_frame_new("unused");
    data_flame[13] = gtk_frame_new("unused");
    data_flame[14] = gtk_frame_new("unused");
    data_flame[15] = gtk_frame_new("unused");
    data_flame[16] = gtk_frame_new("unused");
    data_flame[17] = gtk_frame_new("unused");
    data_flame[18] = gtk_frame_new("unused");
    data_flame[19] = gtk_frame_new("unused");
    data_flame[20] = gtk_frame_new("unused");
    data_flame[21] = gtk_frame_new("unused");
    data_flame[22] = gtk_frame_new("unused");
    data_flame[23] = gtk_frame_new("unused");
    data_flame[24] = gtk_frame_new("unused");
    for(int i = 0;i < 25;i++){
        data_text[i] = gtk_label_new("0.0000");//input上でデータを表示する
        gtk_frame_set_shadow_type(GTK_FRAME(data_flame[i]),GTK_SHADOW_ETCHED_IN);
        gtk_container_add(GTK_CONTAINER(data_flame[i]),data_text[i]);
        gtk_widget_set_size_request(data_flame[i],130,40);
    }
/*
    gtk_widget_set_size_request(odometry,700,30);
    gtk_widget_set_size_request(Pixy1,700,30);
    gtk_widget_set_size_request(Pixy2,700,30);
    gtk_widget_set_size_request(Camera,700,30);*/

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_widget_set_size_request(vbox,650,200);

    for(int j = 0;j < 5;j++){
        hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,2);
        gtk_widget_set_size_request(hbox,650,40);
        gtk_box_pack_start(GTK_BOX(vbox),hbox,true,true,0);
        for(int i = j*5;i < 5 + j*5;i++){
            gtk_box_pack_start(GTK_BOX(hbox),data_flame[i],true,true,0);
        }
    }

    gtk_box_pack_start(GTK_BOX(hbigbox),vbox,true,true,0);

    g_signal_connect(send,"clicked",G_CALLBACK(entryInput),NULL);
    g_signal_connect(run,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(9));

    gtk_widget_show_all(window);
    gtk_widget_show_all(input);

    std::thread ros_loop(ros_main,argc,argv);
    std::thread gtk_loop(run_main);

    ros_loop.join();
    gtk_loop.join();

    return 0;
}

inline void autoMove(){
    constexpr double A_MAX_LOOP[2] = {AMAX[0] / LOOP_RATE,AMAX[1] / LOOP_RATE};
    constexpr double Kp  = 1.4; //自動移動
    constexpr double Ki  = 0.0004;
    constexpr double Kd  = 0.0006;
    static double send_v_x,send_v_y,send_omega;
    static double pid_v_x,pid_v_y,pid_omega;
    static double diff_x,diff_y,diff_yaw;
    static double errer_x,errer_y,errer_omega;
    if(move_data.data.size() != 4){
        move_data.data.resize(4);
    }
    if(move_data.data[0] != 0){
        move_data.data[0] = 0;
    }
    diff_x = goal_x - now_x;
    diff_y = goal_y - now_y;
    diff_yaw = goal_yaw - now_yaw;
    pid_v_x = constrain(diff_x * Kp + errer_x * Ki - now_v_x * Kd,-VMAX[spreaded],VMAX[spreaded]);
    if(fabs(pid_v_x) >= fabs(send_v_x)){
        if(fabs(send_v_x - pid_v_x) > A_MAX_LOOP[spreaded]){
            send_v_x += (pid_v_x >= 0 ? 1 : -1)*A_MAX_LOOP[spreaded];
        }else{
            send_v_x = pid_v_x;
        }
    }else{
        send_v_x = pid_v_x;
        errer_x += diff_x / LOOP_RATE;
    }
    pid_v_y = constrain(diff_y * Kp + errer_y * Ki - now_v_y * Kd,-VMAX[spreaded],VMAX[spreaded]);
    if(fabs(pid_v_y) >= fabs(send_v_y)){
        if(fabs(send_v_y - pid_v_y) > A_MAX_LOOP[spreaded]){
            send_v_y += (pid_v_y >= 0 ? 1 : -1)*A_MAX_LOOP[spreaded];
        }else{
            send_v_y = pid_v_y;
        }
    }else{
        send_v_y = pid_v_y;
        errer_y += diff_y / LOOP_RATE;
    }
    pid_omega = constrain(-diff_yaw / 60 + errer_omega / 5000 - now_omega / 5000,-0.9,0.9);
    if(fabs(pid_omega) >= fabs(send_omega)){
        if(fabs(send_omega - pid_omega) > 0.05){
            send_omega += (pid_omega >= 0 ? 1 : -1)*0.05;
        }else{
            send_omega = pid_omega;
        }
    }else{
        send_omega = pid_omega;
        errer_omega += -diff_yaw / LOOP_RATE;
    }
    if((fabs(diff_x) < 5 && fabs(diff_y) < 5 && fabs(diff_yaw) < 1
                && fabs(now_v_x) < 15 && fabs(now_v_y) < 15 && fabs(now_omega) < 0.02) || skip){
        send_v_x = 0;
        send_v_y = 0;
        send_omega = 0;
        errer_x = 0;
        errer_y = 0;
        errer_omega = 0;
        status = next_move;
        skip = false;
        auto_move = false;
        ROS_INFO("autoMove Fin");
    }
    move_data.data[1] = send_v_x;
    move_data.data[2] = send_v_y;
    move_data.data[3] = send_omega;
    move_pub->publish(move_data);
}

inline void set_auto(double x,double y,double yaw){
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
        move_pub->publish(move_data);
        ros::spinOnce();
    }
    auto_move = true;
}

inline void sendSerial(uint8_t id,uint8_t cmd,int16_t data,ros::Publisher &pub){
    static std_msgs::Int32 send_data;
    send_data.data = (id << 24) + (cmd << 16) + (data & 0xffff);
    pub.publish(send_data);
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
    move_pub->publish(move_data);
    ros::spinOnce();
}

inline void cmd1(int data){
    if(move_data.data.size() != 1){
        move_data.data.resize(1);
    }
    if(move_data.data[0] != data){
        move_data.data[0] = data;
    }
    move_pub->publish(move_data);
    ros::spinOnce();
}
