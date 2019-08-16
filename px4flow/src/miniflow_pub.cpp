#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <px4flow/Flow.h>
#include <string>
#include <string.h>
#include <sstream>
#include <iostream>
#include <fstream> 
#include <vector>

// [need to install serial pkg first!!!!!] sudo apt-get install ros-kinetic-serial
px4flow::Flow miniflow_msg;
serial::Serial ser; //声明串口对象 
double v_xy[2];//速度×高度数据为真实的速度，单位m/s

void data_printing(std::string &buffer){
    std::cout << "data:";
    std::string pkg_tmp;
    const int n = buffer.length();
    int i=0;
    for (i=0;i<n;++i){
        std::cout << "data:"<<std::endl;
        std::cout << (unsigned char)buffer[i];
    }
    std::cout<<"end;" <<std::endl;
}

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "upflow"); 
    //声明节点句柄 
    ros::NodeHandle nh; 

    //发布主题 
    // ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
    ros::Publisher miniflow_pub = nh.advertise<px4flow::Flow>("miniflow", 10);

    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(19200); 
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
    while(ros::ok()) 
    { 

        if(ser.available()){ 
            // ROS_INFO_STREAM("Reading from serial port\n"); 
            // std::string result; 
            // result = ser.read(ser.available()); 
            // // ROS_INFO_STREAM("Read: " << result.data); 
            // std::cout << "data:"<<result<<std::endl;
            // read_pub.publish(result); 
            std::string buffer;
            buffer = ser.read(ser.available());
            {
                int i = 0;
                std::string pkg_tmp;
                while (i < buffer.length()){
                    if ((unsigned char)buffer[i] == 0xFE && (unsigned char)buffer[i+1] == 0x04){
                        //check the pkg
                        pkg_tmp = buffer.substr(i+2,i+8);
                        int16_t tmp;
                        // int16_t time;//光流本次计算与上次计算的时间差
                        double time = 0.0152;
                        uint8_t valid;
                        // time = int16_t((pkg_tmp[4]+pkg_tmp[5]*256)/1000);//单位ms
                        // time = (uint16_t)(pkg_tmp[5]<<8)|(pkg_tmp[4]);
                        // time = time /1000;//单位ms
                        for (int i=0; i<2; ++i){
                            tmp = (int16_t)(pkg_tmp[2*i+1]<<8)|(pkg_tmp[2*i]);
                            v_xy[i] = double(tmp)/(10*time);
                        }
                        valid = pkg_tmp[5];
                        ROS_INFO("v_x=%2.3f,v_y=%2.3f,state:[%1d]",v_xy[0],v_xy[1],(int)valid);
                        miniflow_msg.vx = (float)v_xy[0];
                        miniflow_msg.vy = (float)v_xy[1];
                        miniflow_msg.valid = (int)1;
                        miniflow_msg.quality = (int)valid;
                        miniflow_msg.header.stamp = ros::Time::now ();
                        miniflow_msg.header.frame_id = "FRD";
                        miniflow_pub.publish(miniflow_msg);
                        i = i + 9;
                        break;
                    }else{
                        ++i;        
                    }
                }
            }
        } 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
}
