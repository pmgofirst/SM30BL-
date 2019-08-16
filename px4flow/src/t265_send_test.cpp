#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <nav_msgs/Odometry.h>
#include <string>
#include <string.h>
#include <sstream>
#include <iostream>
#include <fstream> 
#include <vector>
#include <math.h>

// [[[need to install serial pkg first!!!!!]]] sudo apt-get install ros-kinetic-serial

#define SCALE 10000.0f

serial::Serial ser; //声明串口对象 
int pos[3];
int angle[3];
int check_sum;
float float_angle[3];
float w,x,y,z;
char trans_begin0 = (char)(0xFE);
char trans_begin1 = (char)(0x22);
char trans_end = (char)(0x14);
char trans_x[4];
char trans_y[4];
char trans_z[4];
char trans_angle0[4];
char trans_angle1[4];
char trans_angle2[4];
char trans_check_sum[4];
//回调函数
void write_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    x = (float)odom->pose.pose.orientation.x;
    y = (float)odom->pose.pose.orientation.y;
    z = (float)odom->pose.pose.orientation.z;
    w = (float)odom->pose.pose.orientation.w;

    float_angle[0] = (float)atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    float_angle[1] = (float)asin(2*(w*y-x*z));
    float_angle[2] = (float)atan2(2*(w*z+x*y),1-2*(z*z+y*y));

    pos[0] = (int)odom->pose.pose.position.x*SCALE;
    pos[1] = (int)odom->pose.pose.position.y*SCALE;
    pos[2] = (int)odom->pose.pose.position.z*SCALE;

    angle[0] = (int)float_angle[0]*SCALE;
    angle[1] = (int)float_angle[1]*SCALE;
    angle[2] = (int)float_angle[2]*SCALE;

    check_sum = pos[0] + pos[1] + pos[2] + angle[0] + angle[1] + angle[2];

    memcpy(trans_x,&pos[0],sizeof(int));
    memcpy(trans_y,&pos[1],sizeof(int));
    memcpy(trans_z,&pos[2],sizeof(int));

    memcpy(trans_angle0,&angle[0],sizeof(int));
    memcpy(trans_angle1,&angle[1],sizeof(int));
    memcpy(trans_angle2,&angle[2],sizeof(int));

    memcpy(trans_check_sum,&check_sum,sizeof(int));

    ser.write(&trans_begin0);
    ser.write(&trans_begin1);
    ser.write(trans_x);
    ser.write(trans_y);
    ser.write(trans_z);
    ser.write(trans_angle0);
    ser.write(trans_angle1);
    ser.write(trans_angle2);
    ser.write(&trans_end);
}

void write_callback_test()
{
    x = (float)0;
    y = (float)0;
    z = (float)0;
    w = (float)1;

    float_angle[0] = (float)atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    float_angle[1] = (float)asin(2*(w*y-x*z));
    float_angle[2] = (float)atan2(2*(w*z+x*y),1-2*(z*z+y*y));

    pos[0] = (int)12.21312*SCALE;
    pos[1] = (int)21.32421*SCALE;
    pos[2] = (int)123.2123*SCALE;

    angle[0] = (int)float_angle[0]*SCALE;
    angle[1] = (int)float_angle[1]*SCALE;
    angle[2] = (int)float_angle[2]*SCALE;

    check_sum = pos[0] + pos[1] + pos[2] + angle[0] + angle[1] + angle[2];

    memcpy(trans_x,&pos[0],sizeof(int));
    memcpy(trans_y,&pos[1],sizeof(int));
    memcpy(trans_z,&pos[2],sizeof(int));

    memcpy(trans_angle0,&angle[0],sizeof(int));
    memcpy(trans_angle1,&angle[1],sizeof(int));
    memcpy(trans_angle2,&angle[2],sizeof(int));

    memcpy(trans_check_sum,&check_sum,sizeof(int));

    ser.write(&trans_begin0);
    ser.write(&trans_begin1);
    ser.write(trans_x);
    ser.write(trans_y);
    ser.write(trans_z);
    ser.write(trans_angle0);
    ser.write(trans_angle1);
    ser.write(trans_angle2);
    ser.write(&trans_end);
}

int main (int argc, char** argv) 
{ 

    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyS0"); 
        ser.setBaudrate(57600); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    //指定循环的频率 
    ros::Rate loop_rate(50); 

    //初始化节点 
    ros::init(argc, argv, "t265"); 
    //声明节点句柄 
    ros::NodeHandle nh; 


    while(ros::ok()) 
    { 
        //订阅主题，并配置回调函数 
        // ros::Subscriber write_sub = nh.subscribe("/camera/odom/sample", 2, write_callback);
        // ros::spinOnce();
        write_callback_test();
        loop_rate.sleep(); 
    } 
}
