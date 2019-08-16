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

#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )

serial::Serial ser; //声明串口对象 
int pos[3];
int angle[3];
int check_sum;
float float_angle[3];
float w,x,y,z;
unsigned char trans_data[31];

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

    trans_data[0] = 0xFE;
    trans_data[1] = 0x22;
    trans_data[30] = 0x14;

    for(int i = 0;i<3;i++){
        trans_data[3+4*i] = BYTE0(pos[i]);
        trans_data[4+4*i] = BYTE1(pos[i]);
        trans_data[5+4*i] = BYTE2(pos[i]);
        trans_data[6+4*i] = BYTE3(pos[i]);
        trans_data[15+4*i] = BYTE0(angle[i]);
        trans_data[16+4*i] = BYTE1(angle[i]);
        trans_data[17+4*i] = BYTE2(angle[i]);
        trans_data[18+4*i] = BYTE3(angle[i]);
    }
    trans_data[27] = BYTE0(check_sum);
    trans_data[28] = BYTE1(check_sum);
    trans_data[29] = BYTE2(check_sum);
    trans_data[30] = BYTE3(check_sum);
    ser.write(trans_data,31);
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
        ros::Subscriber write_sub = nh.subscribe("/camera/odom/sample", 2, write_callback); 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
}
