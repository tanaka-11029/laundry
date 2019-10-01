#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <thread>
#include <gtk/gtk.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include "raspi_laundry/PrintStatus.h"
#include "cs_connection/RsDataMsg.h"

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
    {"<span foreground='green' size='160000' weight='ultrabold'>展開動作</span>"},
    {"<span foreground='green' size='160000' weight='ultrabold'>収納動作</span>"}
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

constexpr char spread_name[3][20] = {
    {"展開前"},
    {"第一展開"},
    {"第二展開"}
};

constexpr char order_name[4][20] = {
    {"指令なし"},
    {"縮小指令"},
    {"第一展開指令"},
    {"第二展開指令"}
};

constexpr int LOOP_RATE  = 10;
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

GtkWidget* data_text[20];//input上でデータを表示する

std_msgs::Float32MultiArray move_data;
std_msgs::Int32 mechanism_data;
ros::Publisher *move_pub;
ros::Publisher mdd;
ros::Publisher operation;
ros::Publisher mechanism;
char label_name[200];//sprintfのための変数
bool warn = false;
bool towel[2] = {true,true};
bool seats[2] = {true,true};
bool lock = false;
bool ready = false;
bool manual_move = false;
bool only_move = false;
bool coat,fight;
bool emergency = false;
int spreaded = 3;
int spread_move = 0;
int stm_status = 0;
int warn_count = 0;
int wait_num = 0;
double manual_v_x = 0,manual_v_y = 0,manual_omega = 0;

void button_click(GtkWidget* widget,gpointer data);
inline void sendSerial(uint8_t id,uint8_t cmd,int16_t data,ros::Publisher &pub);
inline void cmdnum(double cmd,double x,double y,double omega);
inline void cmd1(int data);

void getResponse(const std_msgs::Int32 &data){
    if(data.data == wait_num){
        wait_num = 0;
        gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
        gtk_label_set_markup(GTK_LABEL(tips),tips_msg[0]);
        switch (data.data){
            case 1://タオル１
                towel[0] = false;
                break;
            case 2://タオル２
                towel[1] = false;
                break;
            case 3://シーツ始め
                seats[0] = false;
                break;
            case 4://シーツ終わり
                seats[1] = false;
                break;
            case 5://第一展開
                break;
            case 6://第二展開
                break;
            case 7://収納
                break;
        }
    }
}

void changeText(const raspi_laundry::PrintStatus &data){
    static int last_status = 0;
    static char label_name[2][200];
    sprintf(label_name[0],"<span foreground='black' size='70000' weight='ultrabold'>ステータス:%d  次:%d</span>",data.status,data.next);
    gtk_label_set_markup(GTK_LABEL(status_num),label_name[0]);
    sprintf(label_name[1],"%s%s",coat_msg[data.coat],fight_msg[data.fight]);
    gtk_label_set_markup(GTK_LABEL(COAT),label_name[1]);
    if(!only_move){
        gtk_label_set_markup(GTK_LABEL(move_mode[0]),move_msg[0][data.towel1]);
        gtk_label_set_markup(GTK_LABEL(move_mode[1]),move_msg[1][data.towel2]);
        gtk_label_set_markup(GTK_LABEL(move_mode[2]),move_msg[2][data.seat]);
        seats[0] = data.seat;
        seats[1] = false;
        towel[0] = data.towel1;
        towel[1] = data.towel2;
    }
    coat = data.coat;
    fight = data.fight;
    if(data.status != last_status){
        last_status = data.status;
        switch (data.status) {
            case 1:
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[7]);
                break;
            case 3://展開動作
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[10]);
                break;
            case 5://バスタオルを干す位置に移動する
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[4]);
                break;
            case 7://バスタオルかける
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[6]);
                break;
            case 10:
                //干し始め位置へ移動
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[3]);
                break;
            case 12:
            case 15://干し終わり動作
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[5]);
                break;
            case 16://帰れる位置に移動する
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[7]);
                break;
            case 17://帰り準備
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[11]);
                break;
            case 18://スタートゾーンへ戻る
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[8]);
                break;
            case 21:
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[0]);
                break;
        }
    }
}

void getData(const std_msgs::Float32MultiArray &place){
    static int i,j;
    static char label_name[8][20];
    if(j > 10){
        j = 0;
        for(i = 0;i < 8;i++){
            sprintf(label_name[i],"%f",place.data[i]);
            gtk_label_set_text(GTK_LABEL(data_text[i]),label_name[i]);
        }
    }else{
        j++;
    }
    if(warn_count != 0){
        warn_count = 0;
    }
}

void getRsmsg(const cs_connection::RsDataMsg &data){
    static int j;
    static char name[3][20];
    if(j > 5){
        j = 0;
        sprintf(name[0],"%f",data.x_distance);
        gtk_label_set_text(GTK_LABEL(data_text[12]),name[0]);
        sprintf(name[1],"%f",data.y_distance);
        gtk_label_set_text(GTK_LABEL(data_text[13]),name[1]);
        sprintf(name[2],"%f",data.z_distance);
        gtk_label_set_text(GTK_LABEL(data_text[14]),name[2]);
    }else{
        j++;
    }
}

void getSwitch(const std_msgs::Int8 &data){
    if(emergency != (data.data & 0x01)){
        emergency = data.data & 0x01;
        button_click(NULL,GINT_TO_POINTER(7));
        gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[emergency]);
    }
}

void getStart(const std_msgs::Bool &data){
    button_click(NULL,GINT_TO_POINTER(9));
}

void getSpread(const std_msgs::Int32 &data){
    //int spread = data.data >> 8;
    //ROS_INFO("spread:%d\t%d\t%d",spread,data.data,data.data >> 8);
    if((data.data >> 8) != spreaded){
        spreaded = data.data >> 8;
        gtk_label_set_text(GTK_LABEL(data_text[11]),spread_name[spreaded]);
    }
}

void getLidar(const std_msgs::Int32 &data){
    static int i;
    static char name[20];
    if(i > 30){
        i = 0;
        sprintf(name,"%d",data.data);
        gtk_label_set_text(GTK_LABEL(data_text[16]),name);
    }else{
        i++;
    }
}

static void ros_main(int argc,char **argv){
    ros::init(argc, argv, "gui_operator");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    ros::Subscriber place = nh.subscribe("place",100,getData);
    ros::Subscriber gui_status = nh.subscribe("print_status",100,changeText);
    ros::Subscriber mechanism_response = nh.subscribe("mechanism_response",100,getResponse);
    ros::Subscriber rs_sub = nh.subscribe("rs_msg",100,getRsmsg);
    ros::Subscriber start_sub = nh.subscribe("start_switch",100,getStart);
    ros::Subscriber spread_sub = nh.subscribe("mechanism_status",100,getSpread);
    ros::Subscriber lidar_sub = nh.subscribe("lidar",100,getLidar);
    ros::Subscriber switch_sub = nh.subscribe("switch",100,getSwitch);
    mdd = nh.advertise<std_msgs::Int32>("motor_serial",100);
    operation = nh.advertise<std_msgs::Float32MultiArray>("Operation",100);
    mechanism = nh.advertise<std_msgs::Int32>("run_mechanism",100);
    bool last_seats[2];
    bool last_towel[2];
    bool zero = true;
    ROS_INFO("Laundry Node start");
    while(ros::ok()){
        if(manual_move){//マニュアル移動 用修正
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
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
                warn = false;
            }
        }
        if(only_move){
            if(seats[0] != last_seats[0]){
                last_seats[0] = seats[0];
                gtk_label_set_markup(GTK_LABEL(move_mode[2]),move_msg[5][seats[0]]);
            }
            if(seats[1] != last_seats[1]){
                last_seats[1] = seats[1];
                gtk_label_set_markup(GTK_LABEL(move_mode[3]),move_msg[6][seats[1]]);
            }
            if(towel[0] != last_towel[0]){
                last_towel[0] = towel[0];
                gtk_label_set_markup(GTK_LABEL(move_mode[0]),move_msg[0][towel[0]]);
            }
            if(towel[1] != last_towel[1]){
                last_towel[1] = towel[1];
                gtk_label_set_markup(GTK_LABEL(move_mode[1]),move_msg[1][towel[1]]);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    sendSerial(255,255,0,mdd);
    gtk_main_quit();
    exit(0);
}

void button_click(GtkWidget* widget,gpointer data){
    switch (GPOINTER_TO_INT(data)) {
        case 0://コート切り替え
            if(!lock){
                coat = !coat;
                cmdnum(12,coat,fight,0);
            }
            break;
        case 1://予選・決勝切り替え
            if(!lock){
                fight = !fight;
                cmdnum(12,coat,fight,0);
            }
            break;
        case 2://キャリブレーション
            lock = true;
            cmd1(-1);
            gtk_label_set_markup(GTK_LABEL(move_mode[3+only_move]),move_msg[3][lock]);
            break;
        case 3://タオル１
            if(!ready){
                towel[0] = !towel[0];
                if(only_move){
                    if(towel[0]){
                        towel[1] = false;
                        seats[0] = false;
                        seats[1] = false;
                    }
                }else{
                    cmdnum(11,towel[0],towel[1],seats[0]);
                }
            }
            break;
        case 4://タオル２
            if(!ready){
                towel[1] = !towel[1];
                if(only_move){
                    if(towel[1]){
                        towel[0] = false;
                        seats[0] = false;
                        seats[1] = false;
                    }
                }else{
                    cmdnum(11,towel[0],towel[1],seats[0]);
                }
            }
            break;
        case 5://シーツ
            if(!ready){
                seats[0] = !seats[0];
                if(only_move){
                    if(seats[0]){
                        towel[0] = false;
                        towel[1] = false;
                        seats[1] = false;
                    }
                }else{
                    cmdnum(11,towel[0],towel[1],seats[0]);
                    seats[1] = 0;
                }
            }
            break;
        case 6://スタート
            manual_move = false;
            gtk_label_set_markup(GTK_LABEL(move_mode[4+only_move]),move_msg[4][manual_move]);
            ready = true;
            if(only_move){
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[5]);
            }else{
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[2]);
                cmd1(-3);//ready信号
            }
            break;
        case 7://ストップ
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[3]);
            gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
            cmd1(-2);
            ready = false;
            if(only_move){
                mechanism_data.data = 0;
                mechanism.publish(mechanism_data);
            }
            break;
        case 8://リセット
            ready = false;
            cmd1(-6);
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[0]);
            gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
            if(only_move){
                mechanism_data.data = 0;
                mechanism.publish(mechanism_data);
            }
            break;
        case 9://コマンド実行
            if(ready){
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[1]);
                ready = false;
                if(only_move){
                    if(towel[0]){//タオルノード起動(1)
                        gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[6]);
                        wait_num = 1;
                        mechanism_data.data = 1;
                        mechanism.publish(mechanism_data);
                    }else if(towel[1]){//タオルノード起動(2)
                        gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[6]);
                        wait_num = 2;
                        mechanism_data.data = 2;
                        mechanism.publish(mechanism_data);
                    }else if(seats[0]){//シーツはじめの動き
                        gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[5]);
                        wait_num = 3;
                        mechanism_data.data = 3;
                        mechanism.publish(mechanism_data);
                    }else if(seats[1]){//シーツ終わりの動き
                        gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[5]);
                        wait_num = 4;
                        mechanism_data.data = 4;
                        mechanism.publish(mechanism_data);
                    }else if(spread_move > 1){
                        spread_move = 0;
                        gtk_label_set_text(GTK_LABEL(data_text[15]),order_name[spread_move]);
                        gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[10]);//展開動作
                        wait_num = 5 + spread_move;
                        mechanism_data.data = wait_num;
                        mechanism.publish(mechanism_data);
                    }else if(spread_move == 1){
                        spread_move = 0;
                        gtk_label_set_text(GTK_LABEL(data_text[15]),order_name[spread_move]);
                        gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[11]);//収納動作
                        wait_num = 7;
                        mechanism_data.data = wait_num;
                        mechanism.publish(mechanism_data);
                    }
                }else{
                    cmd1(-5);
                }
            }
            break;
        case 10://ロック解除
            lock = false;
            gtk_label_set_markup(GTK_LABEL(move_mode[3+only_move]),move_msg[3][lock]);
            break;
        case 11://手動操作
            manual_move = !manual_move;
            gtk_label_set_markup(GTK_LABEL(move_mode[4+only_move]),move_msg[4][manual_move]);
            break;
        case 12://シーツ終わり
            if(!ready && only_move){
                seats[1] = !seats[1];
                if(seats[1]){
                    towel[0] = false;
                    towel[1] = false;
                    seats[0] = false;
                }
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
            case 20:
                button_click(widget,GINT_TO_POINTER((int)x));
                break;
            case 21:
                sendSerial((int)x,(int)y,(int)yaw,mdd);
                break;
            case 22:
                spread_move = (int)x;//展開司令
                gtk_label_set_text(GTK_LABEL(data_text[15]),order_name[spread_move]);
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
                operation.publish(move_data);
                break;
        }
    }else{
        if(cmd == -7 && !only_move){
            only_move = true;
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
        }else if(cmd == -8 && only_move){
            only_move = false;
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
            mechanism_data.data = 0;
            mechanism.publish(mechanism_data);
        }else if(cmd == -9){
            gtk_main_quit();
            exit(0);
        }else{
            move_data.data.resize(1);
            move_data.data[0] = cmd;
            operation.publish(move_data);
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
        case 'k'://シーツ終わり
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

    GtkWidget* vbox;//描画補助ボックス
    GtkWidget* hbox;
    GtkWidget* vbigbox;
    GtkWidget* hbigbox;

    GtkWidget* data_flame[20];

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

    gtk_box_pack_start(GTK_BOX(vbigbox),send,true,true,0);

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
    data_flame[12] = gtk_frame_new("rs_x");
    data_flame[13] = gtk_frame_new("rs_y");
    data_flame[14] = gtk_frame_new("rs_z");
    data_flame[15] = gtk_frame_new("展開司令");
    data_flame[16] = gtk_frame_new("Lidar");
    data_flame[17] = gtk_frame_new("unused");
    data_flame[18] = gtk_frame_new("unused");
    data_flame[19] = gtk_frame_new("unused");
    data_flame[20] = gtk_frame_new("unused");
    for(int i = 0;i < 20;i++){
        data_text[i] = gtk_label_new("0.000000");//input上でデータを表示する
        gtk_frame_set_shadow_type(GTK_FRAME(data_flame[i]),GTK_SHADOW_ETCHED_IN);
        gtk_container_add(GTK_CONTAINER(data_flame[i]),data_text[i]);
        gtk_widget_set_size_request(data_flame[i],130,40);
    }

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_widget_set_size_request(vbox,650,200);

    for(int j = 0;j < 5;j++){
        hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,2);
        gtk_widget_set_size_request(hbox,650,40);
        gtk_box_pack_start(GTK_BOX(vbox),hbox,true,true,0);
        for(int i = j*4;i < 4 + j*4;i++){
            gtk_box_pack_start(GTK_BOX(hbox),data_flame[i],true,true,0);
        }
    }

    gtk_box_pack_start(GTK_BOX(hbigbox),vbox,true,true,0);

    g_signal_connect(send,"clicked",G_CALLBACK(entryInput),NULL);

    gtk_widget_show_all(window);
    gtk_widget_show_all(input);

    std::thread ros_loop(ros_main,argc,argv);
    std::thread gtk_loop(run_main);

    ros_loop.join();
    gtk_loop.join();

    return 0;
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
    operation.publish(move_data);
    ros::spinOnce();
}

inline void cmd1(int data){
    if(move_data.data.size() != 1){
        move_data.data.resize(1);
    }
    if(move_data.data[0] != data){
        move_data.data[0] = data;
    }
    operation.publish(move_data);
    ros::spinOnce();
}
