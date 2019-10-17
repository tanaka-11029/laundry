#include <iostream>
#include <libpixyusb2.h>
#include <ros/ros.h>
#include <laundry/PixyData.h>

Pixy2 pixy;
laundry::PixyData pixy_data;
bool enabled = true;

void  get_line_features()
{
    int  Element_Index;

    // Query Pixy for line features //
    pixy.line.getAllFeatures();

    // Were vectors detected? //
    if (pixy.line.numVectors)
    {
        // Blocks detected - print them! //

        printf ("Detected %d vectors(s)\n", pixy.line.numVectors);

        pixy_data.line_start_x = pixy.line.vectors[0].m_x0;
        pixy_data.line_start_y = pixy.line.vectors[0].m_y0;
        pixy_data.line_end_x = pixy.line.vectors[0].m_x1;
        pixy_data.line_end_y = pixy.line.vectors[0].m_y1;
        pixy_data.line_index = pixy.line.vectors[0].m_index;
        pixy_data.line_flags = pixy.line.vectors[0].m_flags;

        for (Element_Index = 0; Element_Index < pixy.line.numVectors; ++Element_Index)
        {
            printf ("  Vector %d: ", Element_Index + 1);
            pixy.line.vectors[Element_Index].print();
        }
    }

    pixy_data.num_intersections = pixy.line.numIntersections;
    // Were intersections detected? //
    if (pixy.line.numIntersections)
    {
        // Intersections detected - print them! //

        printf ("Detected %d intersections(s)\n", pixy.line.numIntersections);

        pixy_data.intersection_x = pixy.line.intersections[0].m_x;
        pixy_data.intersection_y = pixy.line.intersections[0].m_y;
        pixy_data.intersection_index = pixy.line.intersections[0].m_intLines[0].m_index;
        pixy_data.intersection_angle = pixy.line.intersections[0].m_intLines[0].m_angle;

        for (Element_Index = 0; Element_Index < pixy.line.numIntersections; ++Element_Index)
        {
            printf ("  ");
            pixy.line.intersections[Element_Index].print();
        }
    }

    // Were barcodes detected? //
    if (pixy.line.numBarcodes)
    {
        // Barcodes detected - print them! //

        printf ("Detected %d barcodes(s)\n", pixy.line.numBarcodes);

        for (Element_Index = 0; Element_Index < pixy.line.numBarcodes; ++Element_Index)
        {
            printf ("  Barcode %d: ", Element_Index + 1);
            pixy.line.barcodes[Element_Index].print();
        }
    }
}

void getData(const laundry::PixyData &data){
    enabled = data.enabled;
    pixy.setLamp(data.lamp,0);
    pixy.setServos(data.servo_x,data.servo_y);
}

int main(int argc,char **argv){
    ros::init(argc,argv,"pixy2_node");
    ros::NodeHandle nh;
    ros::Publisher pixy_pub = nh.advertise<laundry::PixyData>("pixy_data",100);
    ros::Subscriber set_mode = nh.subscribe("pixy_set",100,getData);
    ros::Rate loop_rate(100);

    int result;

    //Pixyを開く
    result = pixy.init();
    if(result < 0){
        ROS_ERROR("Faild open Pixy2 %d",result);
        return result;
    }else{
        ROS_INFO("Success open Pixy2");
    }

    //Pixyバージョン確認
    result = pixy.getVersion();
    if(result < 0){
        ROS_ERROR("pixy.getVersion() faild %d",result);
        return result;
    }else{
        pixy.version->print();
    }

    pixy.changeProg("line");
    pixy.setLamp(0,0);
    pixy.setServos(500,500);
    //pixy.line.setMode(LINE_MODE_WHITE_LINE);//白線を感知

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        if(enabled){
            get_line_features();
            pixy_pub.publish(pixy_data);
        }
    }

    pixy.setLamp(0,0);
    return 0;

}

