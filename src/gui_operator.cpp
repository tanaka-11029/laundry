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
#include "cs_connection/PrintStatus.h"
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
    {"<span foreground='LimeGreen' size='140000' weight='ultrabold'>動作終了</span>"},//ホームへ移動
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
    {"<span foreground='black' size='70000' weight='ultrabold'>全体動作待機中です。\nスペースキーを\n押してください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>停止しました。\n再度動作を設定\nしてください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>STM32からの信号を\n受信できません。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>機構のみ動作待機中\nです。スペースキーを\n押してください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>準備動作待機中です。\nスペースキーを\n押してください。</span>"}
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

constexpr char lock_name[2][20] = {
    {"操作ロック(P)"},
    {"ロック解除(P)"}
};

constexpr int LOOP_RATE  = 10;
constexpr double AMAX[2] = {300,50}; // mm/s/s
constexpr double VMAX[2] = {1500,250}; // mm/s

GtkWidget* window;//Windowを作る

GtkWidget* status_num;//Window上で描画する文字
GtkWidget* STATUS;
GtkWidget* COAT;
GtkWidget* tips;
GtkWidget* move_mode[6];

GtkWidget* unlock;
GtkWidget* sheet_on;
GtkWidget* sheet_off;//シーツ機構ごと
GtkWidget* start;
GtkWidget* hbox_sheet;
GtkWidget* hbox_move;
GtkWidget* only_mechanism;

std_msgs::Float32MultiArray move_data;
std_msgs::Int32 mechanism_data;
ros::Publisher *move_pub;
ros::Publisher mdd;
ros::Publisher operation;
ros::Publisher mechanism;
char label_name[200];//sprintfのための変数
bool towel[2] = {true,true};
bool seats[2] = {true,true};
bool lock = false;
bool ready = false;
bool manual_move = false;
bool only_move = false;
bool coat,fight;
bool emergency = false;
bool setup_move = false;
int spread_move = 0;
int wait_num = 0;
double manual_v_x = 0,manual_v_y = 0,manual_omega = 0;

void button_click(GtkWidget* widget,gpointer data);
inline void sendSerial(uint8_t id,uint8_t cmd,int16_t data);
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

void changeText(const cs_connection::PrintStatus &data){
    static int last_status = 0,last_next = 0;
    static char label_name[2][200];
    static bool warn = false;
    //static bool last_coat,last_fight;
    if(data.next != last_next || data.status != last_status){
        last_next = data.next;
        sprintf(label_name[0],"<span foreground='black' size='70000' weight='ultrabold'>ステータス:%d  次:%d</span>",data.status,data.next);
        gtk_label_set_markup(GTK_LABEL(status_num),label_name[0]);
    }
    if(data.coat != coat || data.fight != fight){
        sprintf(label_name[1],"%s%s",coat_msg[data.coat],fight_msg[data.fight]);
        gtk_label_set_markup(GTK_LABEL(COAT),label_name[1]);
        coat = data.coat;
        fight = data.fight;
    }
    if(data.warn != warn){
        warn = data.warn;
        if(warn){
            //button_click(NULL,GINT_TO_POINTER(7));
            gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[9]);
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[4]);
        }else{
            //button_click(NULL,GINT_TO_POINTER(7));
            gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[3]);
        }
    }
    if(!only_move){
        seats[0] = data.seat;
        towel[0] = data.towel1;
        towel[1] = data.towel2;
    }
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

void getSwitch(const std_msgs::Int8 &data){
    if(emergency != (data.data & 0x01)){
        emergency = data.data & 0x01;
        //button_click(NULL,GINT_TO_POINTER(7));
        gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[emergency]);
    }
}

void getStart(const std_msgs::Bool &data){
    if(!emergency){
        button_click(NULL,GINT_TO_POINTER(9));
    }
}

static void ros_main(int argc,char **argv){
    ros::init(argc, argv, "gui_operator");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    ros::Subscriber gui_status = nh.subscribe("print_status",100,changeText);
    ros::Subscriber mechanism_response = nh.subscribe("mechanism_response",100,getResponse);
    ros::Subscriber start_sub = nh.subscribe("start_switch",100,getStart);
    ros::Subscriber switch_sub = nh.subscribe("switch",100,getSwitch);
    mdd = nh.advertise<std_msgs::Int32>("motor_serial",100);
    operation = nh.advertise<std_msgs::Float32MultiArray>("Operation",100);
    mechanism = nh.advertise<std_msgs::Int32>("run_mechanism",100);
    bool last_seats[2];
    bool last_towel[2];
    bool zero = true;
    int first = 0;
    ROS_INFO("GUI Operator start");
    while(ros::ok()){
        if(first < 10){
            cmd1(-4);
            first++;
        }
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
        if(only_move){
            if(seats[0] != last_seats[0]){
                last_seats[0] = seats[0];
                gtk_label_set_markup(GTK_LABEL(move_mode[2]),move_msg[5][seats[0]]);
            }
            if(seats[1] != last_seats[1]){
                last_seats[1] = seats[1];
                gtk_label_set_markup(GTK_LABEL(move_mode[3]),move_msg[6][seats[1]]);
            }
        }else if(seats[0] != last_seats[0]){
            last_seats[0] = seats[0];
            gtk_label_set_markup(GTK_LABEL(move_mode[2]),move_msg[2][seats[0]]);
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
    gtk_main_quit();
    exit(0);
}

void button_click(GtkWidget* widget,gpointer data){
    switch (GPOINTER_TO_INT(data)) {
        case 0://コート切り替え
            if(!lock){
                cmdnum(12,!coat,fight,0);
            }
            break;
        case 1://予選・決勝切り替え
            if(!lock){
                cmdnum(12,coat,!fight,0);
            }
            break;
        case 2://キャリブレーション
            lock = true;
            setup_move = false;
            cmd1(-1);
            gtk_label_set_markup(GTK_LABEL(move_mode[3+only_move]),move_msg[3][lock]);
            break;
        case 3://タオル１
            if(!ready){
                if(only_move){
                    towel[0] = !towel[0];
                    if(towel[0]){
                        towel[1] = false;
                        seats[0] = false;
                        seats[1] = false;
                    }
                }else{
                    cmdnum(11,!towel[0],towel[1],seats[0]);
                }
            }
            break;
        case 4://タオル２
            if(!ready){
                if(only_move){
                    towel[1] = !towel[1];
                    if(towel[1]){
                        towel[0] = false;
                        seats[0] = false;
                        seats[1] = false;
                    }
                }else{
                    cmdnum(11,towel[0],!towel[1],seats[0]);
                }
            }
            break;
        case 5://シーツ
            if(!ready){
                if(only_move){
                    seats[0] = !seats[0];
                    if(seats[0]){
                        towel[0] = false;
                        towel[1] = false;
                        seats[1] = false;
                    }
                }else{
                    cmdnum(11,towel[0],towel[1],!seats[0]);
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
            setup_move = false;
            cmd1(-6);
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[0]);
            gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[0]);
            if(only_move){
                mechanism_data.data = 0;
                mechanism.publish(mechanism_data);
            }
            break;
        case 9://コマンド実行
            if(ready && !emergency){
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[1]);
                ready = false;
                if(setup_move){
                    setup_move = false;
                    gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[10]);//展開動作
                    wait_num = 10+fight;
                    mechanism_data.data = wait_num;
                    mechanism.publish(mechanism_data);
                }else if(only_move){
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
                    }
                }else{
                    cmd1(-5);
                }
            }
            break;
        case 10://ロック解除
            lock = !lock;
            gtk_button_set_label(GTK_BUTTON(unlock), lock_name[lock]);
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
        case 13://最初の準備動作
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[6]);
            setup_move = true;
            ready = true;
            cmd1(-6);
            break;
        case 14://機構動作の切り替え
            if(!lock){
                if (only_move){
                    only_move = false;
                    gtk_button_set_label(GTK_BUTTON(sheet_on), "シーツ(J)");
                    gtk_button_set_label(GTK_BUTTON(start), "スタート(B)");
                    gtk_button_set_label(GTK_BUTTON(only_mechanism),"機構動作");
                    gtk_widget_destroy(sheet_off);
                    for (int i = 2; i < 6; i++){
                        gtk_widget_destroy(move_mode[i]);
                        move_mode[i] = gtk_label_new(NULL);
                    }
                    gtk_label_set_markup(GTK_LABEL(move_mode[2]), move_msg[2][seats[0]]);
                    gtk_label_set_markup(GTK_LABEL(move_mode[3]), move_msg[3][lock]);
                    gtk_label_set_markup(GTK_LABEL(move_mode[4]), move_msg[4][manual_move]);
                    for (int i = 2; i < 5; i++){
                        gtk_box_pack_start(GTK_BOX(hbox_move), move_mode[i], true, true, 0);
                    }
                    gtk_widget_show_all(window);
                    mechanism_data.data = 0;
                    mechanism.publish(mechanism_data);
                } else {
                    only_move = true;
                    towel[0] = false;
                    towel[1] = false;
                    seats[0] = false;
                    seats[1] = false;
                    gtk_button_set_label(GTK_BUTTON(sheet_on), "シーツかけ始め(J)");
                    sheet_off = gtk_button_new_with_label("シーツかけ終わり(K)");
                    gtk_box_pack_start(GTK_BOX(hbox_sheet), sheet_off, true, true, 0);
                    gtk_button_set_label(GTK_BUTTON(start), "機構のみスタート(B)");
                    gtk_button_set_label(GTK_BUTTON(only_mechanism),"全体動作");
                    g_signal_connect(sheet_off, "clicked", G_CALLBACK(button_click), GINT_TO_POINTER(12));
                    for (int i = 2; i < 5; i++){
                        gtk_widget_destroy(move_mode[i]);
                        move_mode[i] = gtk_label_new(NULL);
                    }
                    move_mode[5] = gtk_label_new(NULL);
                    gtk_label_set_markup(GTK_LABEL(move_mode[2]), move_msg[5][seats[0]]);
                    gtk_label_set_markup(GTK_LABEL(move_mode[3]), move_msg[6][seats[1]]);
                    gtk_label_set_markup(GTK_LABEL(move_mode[4]), move_msg[3][lock]);
                    gtk_label_set_markup(GTK_LABEL(move_mode[5]), move_msg[4][manual_move]);
                    for (int i = 2; i < 6; i++){
                        gtk_box_pack_start(GTK_BOX(hbox_move), move_mode[i], true, true, 0);
                    }
                    gtk_widget_show_all(window);
                }
            }
            break;
    }
}

void key_press(GtkWidget* widget,GdkEventKey *event,gpointer data){
    //printf("key_press:keyval=%d\n",event->keyval);
    switch (event->keyval) {
        case 't'://コート切り替え
        case 'T':
            button_click(widget,GINT_TO_POINTER(0));
            break;
        case 'y'://予選・決勝切り替え
        case 'Y':
            button_click(widget,GINT_TO_POINTER(1));
            break;
        case 'u'://キャリブレーション
        case 'U':
            button_click(widget,GINT_TO_POINTER(2));
            break;
        case 'g'://タオル１
        case 'G':
            button_click(widget,GINT_TO_POINTER(3));
            break;
        case 'h'://タオル２
        case 'H':
            button_click(widget,GINT_TO_POINTER(4));
            break;
        case 'j'://シーツ
        case 'J':
            button_click(widget,GINT_TO_POINTER(5));
            break;
        case 'b'://スタート
        case 'B':
            button_click(widget,GINT_TO_POINTER(6));
            break;
        case 'n'://ストップ
        case 'N':
            button_click(widget,GINT_TO_POINTER(7));
            break;
        case 'm'://リセット
        case 'M':
            button_click(widget,GINT_TO_POINTER(8));
            break;
        case ' '://コマンド実行
            button_click(widget,GINT_TO_POINTER(9));
            break;
        case 'p'://ロック解除
        case 'P':
            button_click(widget,GINT_TO_POINTER(10));
            break;
        case 'r'://手動操作
        case 'R':
            button_click(widget,GINT_TO_POINTER(11));
            break;
        case 'k'://シーツ終わり
        case 'K':
            button_click(widget,GINT_TO_POINTER(12));
            break;
        case 'v':
        case 'V':
            button_click(widget,GINT_TO_POINTER(13));
            break;
        case 'w':
        case 'W':
            manual_v_y = 500;
            break;
        case 'a':
        case 'A':
            manual_v_x = -500;
            break;
        case 's':
        case 'S':
            manual_v_y = -500;
            break;
        case 'd':
        case 'D':
            manual_v_x = 500;
            break;
        case 'e':
        case 'E':
            manual_omega = 0.3;
            break;
        case 'q':
        case 'Q':
            manual_omega = -0.3;
            break;
    }
}

void key_release(GtkWidget* widget,GdkEventKey *event,gpointer data){
    switch (event->keyval){
        case 'w':
        case 'W':
        case 's':
        case 'S':
            manual_v_y = 0;
            break;
        case 'a':
        case 'A':
        case 'd':
        case 'D':
            manual_v_x = 0;
            break;
        case 'e':
        case 'E':
        case 'q':
        case 'Q':
            manual_omega = 0;
            break;
    }
}

static void run_main(){
    gtk_main();
}

void quit_main(GtkWidget *button,gpointer user_data){
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
    GtkWidget* setup = gtk_button_new_with_label("準備動作(V)");
    start = gtk_button_new_with_label("スタート(B)");
    GtkWidget* stop = gtk_button_new_with_label("停止(N)");
    GtkWidget* reset = gtk_button_new_with_label("始めに戻る(M)");
    GtkWidget* manual = gtk_button_new_with_label("手動操作(R)");
    unlock = gtk_button_new_with_label("操作ロック(P)");
    sheet_on = gtk_button_new_with_label("シーツ(J)");
    sheet_off = gtk_button_new_with_label("シーツかけ終わり(K)");
    only_mechanism = gtk_button_new_with_label("機構動作");

    GtkWidget* vbox;//描画補助ボックス
    GtkWidget* hbox;
    GtkWidget* vbigbox;
    GtkWidget* hbigbox;

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);//Windowの描画
    gtk_window_set_position(GTK_WINDOW(window),GTK_WIN_POS_CENTER);
    gtk_window_set_title(GTK_WINDOW(window),"GUI Operator");
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

    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,5);
    gtk_box_pack_start(GTK_BOX(hbox),setup,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),start,true,true,0);

    gtk_box_pack_start(GTK_BOX(vbox),hbox,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbigbox),vbox,true,true,0);


    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_widget_set_size_request(vbox,300,200);
    gtk_box_pack_start(GTK_BOX(vbox),fight_mode,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),towel2,true,true,0);

    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,5);
    gtk_box_pack_start(GTK_BOX(hbox),stop,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),unlock,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),only_mechanism,true,true,0);

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
    g_signal_connect(setup,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(13));
    g_signal_connect(only_mechanism,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(14));

    gtk_widget_show_all(window);

    std::thread ros_loop(ros_main,argc,argv);
    std::thread gtk_loop(run_main);

    ros_loop.join();
    gtk_loop.join();

    return 0;
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
