#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <gtk/gtk.h>

using namespace std;

constexpr char status_msg[][100] = {
    {"<span foreground='orange' size='140000' weight='ultrabold'>停止中</span>"},
    {"<span foreground='red' size='140000' weight='ultrabold'>非常停止</span>"},
    {"<span foreground='brown' size='140000' weight='ultrabold'>エラー</span>"},
    {"<span foreground='green' size='120000' weight='ultrabold'>シーツへ移動</span>"},
    {"<span foreground='green' size='120000' weight='ultrabold'>タオルへ移動</span>"},
    {"<span foreground='purple' size='120000' weight='ultrabold'>シーツ干し中</span>"},
    {"<span foreground='purple' size='120000' weight='ultrabold'>タオル干し中</span>"}
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
    {"<span foreground='black' size='70000' weight='ultrabold'>装填が終わったら\nSキーを押して\nください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>動作を完了するまで\nお待ちください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>全体動作待機中\nです。スペースキーを\n押してください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>シーツ動作待機中\nです。スペースキーを\n押してください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>タオル動作待機中\nです。スペースキーを\n押してください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>帰還動作待機中\nです。スペースキーを\n押してください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>その他動作待機中\nです。スペースキーを\n押してください。</span>"},
    {"<span foreground='black' size='70000' weight='ultrabold'>停止しました。\nsキーで再開します。</span>"}
};

GtkWidget* status_num;//Window上で描画する文字
GtkWidget* STATUS;
GtkWidget* COAT;
GtkWidget* tips;

GtkWidget* Gcmd;//Input上で入力を受け付けるエントリー
GtkWidget* Gx;
GtkWidget* Gy;
GtkWidget* Gyaw;

GtkWidget* odometry;//input上でデータを表示する
GtkWidget* Pixy1;
GtkWidget* Pixy2;
GtkWidget* Camera;

char label_name[200];//sprintfのための変数
bool coat,fight,wait = true;
int status,Next = 1;
int move_set = 0;
double now_x,now_y,now_yaw,now_Vx,now_Vy,now_Vt,Vx,Vy,Vt;

static void ros_main(int argc,char **argv){
    ros::init(argc, argv, "laundry_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    int last_status,last_next;
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        switch (status) {
            case 0:
                if(now_Vx + now_Vy + now_Vt + Vx + Vy + Vt + wait == 0 && move_set != 0){
                    status = Next;
                }else{
                    wait = true;
                }
                break;
            case 1:
                break;
        }
        if(last_status != status || last_next != Next){
            sprintf(label_name,"<span foreground='black' size='70000' weight='ultrabold'>ステータス:%d  次:%d</span>",status,Next);
            gtk_label_set_markup(GTK_LABEL(status_num),label_name);
            last_status = status;
            last_next = Next;
            if(status == 0);//非常停止
        }
    }
    gtk_main_quit();
    exit(0);
}

void button_click(GtkWidget* widget,gpointer data){
    switch (GPOINTER_TO_INT(data)) {
        case 0://キャリブレーション
            break;
        case 1://コート切り替え
            coat = !coat;
            sprintf(label_name,"%s%s",coat_msg[coat],fight_msg[fight]);
            gtk_label_set_markup(GTK_LABEL(COAT),label_name);
            break;
        case 2://予選・決勝切り替え
            fight = !fight;
            sprintf(label_name,"%s%s",coat_msg[coat],fight_msg[fight]);
            gtk_label_set_markup(GTK_LABEL(COAT),label_name);
            break;
        case 3://スタート
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[2]);
            move_set = 1;
            break;
        case 4://ストップ
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[7]);
            Next = status;
            status = 0;
            break;
        case 5://ホームに帰る
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[5]);
            move_set = 4;
            break;
        case 6://シーツ
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[3]);
            move_set = 2;
            break;
        case 7://タオル
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[4]);
            move_set = 3;
            break;
        case 8://リセット
            status = 0;
            Next = 1;
            move_set = 0;
            wait = true;
            gtk_label_set_markup(GTK_LABEL(tips),tips_msg[0]);
            break;
        case 9://コマンド実行
            if(move_set != 0){
                wait = false;
                gtk_label_set_markup(GTK_LABEL(tips),tips_msg[1]);
            }
            break;
    }
}

void get_data(GtkWidget* widget,gpointer data){
    int cmd;
    double x,y,yaw;
    char temp[100];
    const gchar *text;
    text = gtk_entry_get_text(GTK_ENTRY(Gcmd));
    sprintf(temp,"%s",text);
    cmd = atoi(temp);
    if(cmd >= 10){
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
                break;
            case 30:
                gtk_label_set_markup(GTK_LABEL(STATUS),status_msg[(int)x]);
                break;
        }
    }else{
        if(cmd >= 0)button_click(widget,GINT_TO_POINTER(cmd));
        else if(cmd == -1){
            gtk_main_quit();
            exit(0);
        }
        x = 0;
        y = 0;
        yaw = 0;
        printf("%d : %lf %lf %lf\n",cmd,x,y,yaw);
    }
}

void key_press(GtkWidget* widget,GdkEventKey *event,gpointer data){
    //printf("key_press:keyval=%c\n",event->keyval);
    switch (event->keyval) {
        case 'c'://キャリブレーション
            button_click(widget,GINT_TO_POINTER(0));
            break;
        case 'x'://コート切り替え
            button_click(widget,GINT_TO_POINTER(1));
            break;
        case 'f'://予選・決勝切り替え
            button_click(widget,GINT_TO_POINTER(2));
            break;
        case 's'://スタート
            button_click(widget,GINT_TO_POINTER(3));
            break;
        case 'z'://ストップ
            button_click(widget,GINT_TO_POINTER(4));
            break;
        case 'h'://ホームに帰る
            button_click(widget,GINT_TO_POINTER(5));
            break;
        case 'l'://シーツ
            button_click(widget,GINT_TO_POINTER(6));
            break;
        case 't'://タオル
            button_click(widget,GINT_TO_POINTER(7));
            break;
        case 'r'://リセット
            button_click(widget,GINT_TO_POINTER(8));
            break;
        case ' '://コマンド実行
            button_click(widget,GINT_TO_POINTER(9));
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

    GtkWidget* window;//Windowを作る
    GtkWidget* input;

    GtkWidget* home = gtk_button_new_with_label("スタートゾーンへ帰る(H)");//window上のボタン
    GtkWidget* cali = gtk_button_new_with_label("キャリブレーション(C)");
    GtkWidget* red = gtk_button_new_with_label("コート切り替え(X)");
    GtkWidget* blue = gtk_button_new_with_label("予選/決勝切り替え(F)");
    GtkWidget* start = gtk_button_new_with_label("スタート(S)");
    GtkWidget* stop = gtk_button_new_with_label("停止(Z)");
    GtkWidget* sheet = gtk_button_new_with_label("シーツを干す(L)");
    GtkWidget* towel = gtk_button_new_with_label("タオルを干す(T)");
    GtkWidget* reset = gtk_button_new_with_label("始めに戻る(R)");

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

    GtkWidget* grid;//inputで使うグリッド

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);//Windowの描画
    gtk_window_set_position(GTK_WINDOW(window),GTK_WIN_POS_CENTER);
    gtk_window_set_title(GTK_WINDOW(window),"Laundry");
    gtk_container_set_border_width(GTK_CONTAINER(window),10);
    gtk_widget_set_size_request(window,900,1080);
    g_signal_connect(window,"destroy",G_CALLBACK(quit_main),NULL);
    g_signal_connect(G_OBJECT(window),"key-press-event",G_CALLBACK(key_press),NULL);

    vbigbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_container_add(GTK_CONTAINER(window),vbigbox);

    status_num = gtk_label_new(NULL);//Window上で描画する文字
    STATUS = gtk_label_new(NULL);
    COAT = gtk_label_new(NULL);
    tips = gtk_label_new(NULL);

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

    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,5);//ボタン配置
    gtk_widget_set_size_request(hbox,900,200);
    gtk_box_pack_start(GTK_BOX(vbigbox),hbox,true,true,0);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_widget_set_size_request(vbox,300,200);
    gtk_box_pack_start(GTK_BOX(vbox),cali,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),start,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),sheet,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),vbox,true,true,0);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_widget_set_size_request(vbox,300,200);
    gtk_box_pack_start(GTK_BOX(vbox),red,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),stop,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),towel,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),vbox,true,true,0);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_widget_set_size_request(vbox,300,200);
    gtk_box_pack_start(GTK_BOX(vbox),blue,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),home,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),reset,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbox),vbox,true,true,0);

    gtk_widget_set_size_request(COAT,900,150);//コート
    gtk_box_pack_start(GTK_BOX(vbigbox),COAT,true,true,0);
    gtk_widget_set_size_request(tips,900,400);//説明
    gtk_box_pack_start(GTK_BOX(vbigbox),tips,true,true,0);

    g_signal_connect(cali,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(0));
    g_signal_connect(red,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(1));
    g_signal_connect(blue,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(2));
    g_signal_connect(start,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(3));
    g_signal_connect(stop,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(4));
    g_signal_connect(home,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(5));
    g_signal_connect(sheet,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(6));
    g_signal_connect(towel,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(7));
    g_signal_connect(reset,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(8));

    input = gtk_window_new(GTK_WINDOW_TOPLEVEL);//inputの描画
    gtk_window_set_position(GTK_WINDOW(input),GTK_WIN_POS_CENTER);
    gtk_window_set_title(GTK_WINDOW(input),"Operator");
    gtk_container_set_border_width(GTK_CONTAINER(input),10);
    gtk_widget_set_size_request(input,900,200);
    g_signal_connect(input,"destroy",G_CALLBACK(quit_main),NULL);

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

    odometry = gtk_label_new("X:------\tY:------\tYaw:------");//input上でデータを表示する
    Pixy1 = gtk_label_new("Xs:--\tXe:--\tYs:--\tYe:--\tIx:--\tIy:--");
    Pixy2 = gtk_label_new("Xs:--\tXe:--\tYs:--\tYe:--\tIx:--\tIy:--");
    Camera = gtk_label_new("X:------\tY:------");
/*
    gtk_widget_set_size_request(odometry,700,30);
    gtk_widget_set_size_request(Pixy1,700,30);
    gtk_widget_set_size_request(Pixy2,700,30);
    gtk_widget_set_size_request(Camera,700,30);*/

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);
    gtk_widget_set_size_request(vbox,700,200);
    gtk_box_pack_start(GTK_BOX(vbox),odometry,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),Pixy1,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),Pixy2,true,true,0);
    gtk_box_pack_start(GTK_BOX(vbox),Camera,true,true,0);
    gtk_box_pack_start(GTK_BOX(hbigbox),vbox,true,true,0);

    g_signal_connect(send,"clicked",G_CALLBACK(get_data),NULL);
    g_signal_connect(run,"clicked",G_CALLBACK(button_click),GINT_TO_POINTER(9));

    gtk_widget_show_all(window);
    gtk_widget_show_all(input);

    thread ros_loop(ros_main,argc,argv);
    thread gtk_loop(run_main);

    ros_loop.join();
    gtk_loop.join();

    return 0;
}
