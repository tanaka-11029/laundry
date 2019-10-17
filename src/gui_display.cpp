#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <thread>
#include <gtk/gtk.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include "laundry/PrintStatus.h"
#include "cs_connection/RsDataMsg.h"

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

constexpr int LOOP_RATE  = 5;

GtkWidget* input;
GtkWidget* Gx;
GtkWidget* Gy;
GtkWidget* Gyaw;
GtkWidget* Gcmd;

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
bool setup_move = false;
int spreaded = 3;
int spread_move = 0;
int stm_status = 0;
int warn_count = 0;
int wait_num = 0;
double manual_v_x = 0,manual_v_y = 0,manual_omega = 0;

inline void sendSerial(uint8_t id,uint8_t cmd,int16_t data);

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
    if(j > 10){
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

void getSpread(const std_msgs::Int16 &data){
    static int mechanism;
    static char name[20];
    if((data.data >> 8) != spreaded){
        spreaded = data.data >> 8;
        gtk_label_set_text(GTK_LABEL(data_text[11]),spread_name[spreaded]);
    }
    if(mechanism != (data.data & 0xff)){
        mechanism = data.data & 0xff;
        sprintf(name,"%d",mechanism);
        gtk_label_set_text(GTK_LABEL(data_text[18]),name);
    }
}

void getLidar(const std_msgs::Int64 &data){
    static int i;
    static char name[2][20];
    if(i > 15){
        i = 0;
        sprintf(name[0],"%ld",data.data & 0xffffffff);
        gtk_label_set_text(GTK_LABEL(data_text[16]),name[0]);
        sprintf(name[1],"%ld",(data.data >> 32) & 0xffffffff);
        gtk_label_set_text(GTK_LABEL(data_text[17]),name[1]);
    }else{
        i++;
    }
}

static void ros_main(int argc,char **argv){
    ros::init(argc, argv, "gui_display");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    ros::Subscriber place = nh.subscribe("place",100,getData);
    ros::Subscriber rs_sub = nh.subscribe("rs_msg",100,getRsmsg);
    ros::Subscriber spread_sub = nh.subscribe("mechanism_status",100,getSpread);
    ros::Subscriber lidar_sub = nh.subscribe("lidar",100,getLidar);
    mdd = nh.advertise<std_msgs::Int32>("motor_serial",100);
    operation = nh.advertise<std_msgs::Float32MultiArray>("Operation",100);
    mechanism = nh.advertise<std_msgs::Int32>("run_mechanism",100);
    ROS_INFO("GUI Display start");
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    gtk_main_quit();
    exit(0);
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
            case 21:
                sendSerial((int)x,(int)y,(int)yaw);
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
        if(cmd == -9){
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

void key_release(GtkWidget* widget,GdkEventKey *event,gpointer data){
    switch (event->keyval){
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

    input = gtk_window_new(GTK_WINDOW_TOPLEVEL);//inputの描画
    gtk_window_set_position(GTK_WINDOW(input),GTK_WIN_POS_CENTER);
    gtk_window_set_title(GTK_WINDOW(input),"GUI Display");
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
    data_flame[11] = gtk_frame_new("展開");
    data_flame[12] = gtk_frame_new("rs_x");
    data_flame[13] = gtk_frame_new("rs_y");
    data_flame[14] = gtk_frame_new("rs_z");
    data_flame[15] = gtk_frame_new("展開司令");
    data_flame[16] = gtk_frame_new("Lidar_X");
    data_flame[17] = gtk_frame_new("Lidar_Y");
    data_flame[18] = gtk_frame_new("機構");
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

    gtk_widget_show_all(input);

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