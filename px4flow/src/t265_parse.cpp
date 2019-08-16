#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <string>
#include <string.h>
#include <sstream>
#include <iostream>
#include <fstream> 
#include <vector>
#include <stdio.h>

// [need to install serial pkg first!!!!!] sudo apt-get install ros-kinetic-serial

#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )


#define SCALE 10000.0f
serial::Serial ser; //声明串口对象 
double v_xy[2];//速度×高度数据为真实的速度，单位m/s
//回调函数 
void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
    ser.write(msg->data);   //发送串口数据 
} 

void update_state(std::string &pkg_tmp){
    static int16_t tmp;
    static int16_t time;
    // static int16_t pos[3];//pos[0] = x;pos[1] = y;pos[2] = z;
    static int pos[3];
    // static int16_t angle[3];// angle[0]=roll,angle[1]=pitch,angle[2]=yaw
    static int angle[3];
    static int check;
    static int sum;
    float double_pos[3];
    float double_angle[3];
    bool check_status;

    // pos[0] = (int16_t)(pkg_tmp[0]<<8)|(pkg_tmp[1]);
    // pos[1] = (int16_t)(pkg_tmp[2]<<8)|(pkg_tmp[3]);
    // pos[2] = (int16_t)(pkg_tmp[4]<<8)|(pkg_tmp[5]);

    // angle[0] = (uint16_t)(pkg_tmp[6]<<8)|(pkg_tmp[7]);
    // angle[1] = (uint16_t)(pkg_tmp[8]<<8)|(pkg_tmp[9]);
    // angle[2] = (uint16_t)(pkg_tmp[10]<<8)|(pkg_tmp[11]);

    for (int i=0;i<3;i++){
        // 真实数据
        // pos[i] = (int)(pkg_tmp[4*i]<<24|(pkg_tmp[4*i+1]<<16|(pkg_tmp[4*i+2]<<8|(pkg_tmp[4*i+3]))));
        BYTE3(pos[i]) = pkg_tmp[4*i];
        BYTE2(pos[i]) = pkg_tmp[4*i+1];
        BYTE1(pos[i]) = pkg_tmp[4*i+2];
        BYTE0(pos[i]) = pkg_tmp[4*i+3];
        // angle[i] = (int)(pkg_tmp[4*i+12]<<24|(pkg_tmp[4*i+13]<<16|(pkg_tmp[4*i+14]<<8|(pkg_tmp[4*i+15]))));
        BYTE3(angle[i]) = pkg_tmp[4*i+12];
        BYTE2(angle[i]) = pkg_tmp[4*i+13];
        BYTE1(angle[i]) = pkg_tmp[4*i+14];
        BYTE0(angle[i]) = pkg_tmp[4*i+15];
        // double_pos[i] = (float)(pos[i]/1000.0f);
        double_pos[i] = (float)pos[i]/SCALE;
        double_angle[i] = (float)angle[i]/SCALE;
        // double_angle[i] = (float)(angle[i]/100.0f);
    }
    // check = (int16_t)(pkg_tmp[12]<<8)|(pkg_tmp[13]);
    // check = (int)(pkg_tmp[24]<<24|pkg_tmp[25]<<16|pkg_tmp[26]<<8|pkg_tmp[27]);
        BYTE3(check) = pkg_tmp[24];
        BYTE2(check) = pkg_tmp[25];
        BYTE1(check) = pkg_tmp[26];
        BYTE0(check) = pkg_tmp[27];
    sum = (int)(pos[0]+pos[1]+pos[2]+angle[0]+angle[1]+angle[2]);
    // sum = (int16_t)((double_pos[0]+double_pos[1]+double_pos[2]+double_angle[0]+double_angle[1]+double_angle[2])*1000);

    check_status = (check == sum);  
    ROS_INFO("-------T265---------\n"
    "POS X:[\t%2.4f],Y:[\t%2.4f],Z:[\t%2.4f],\n"
    "Angle Pitch:[\t%2.4f],Roll:[\t%2.4f],Yaw:[\t%2.4f]\n"
    "sum:[\t%5.5f],check:[\t%5.5f],check_status:[%1d]\n"
    "======================================================\n",
    double_pos[0],
    double_pos[1],
    double_pos[2],
    double_angle[0],
    double_angle[1],
    double_angle[2],
    (float)sum/SCALE,
    (float)check/SCALE,
    (int)check_status
    );

}

void data_decoding(){

    static uint8_t data;
    static std::string pkg_tmp;
    while(ros::ok()){
        ser.read(&data,1);
        if (data == 0xFE){
            data = '0';
            ser.read(&data,1);
            if (data == 0x22){
                for(int i = 0;i<28;i++){
                    data = '0';
                    ser.read(&data,1);
                    pkg_tmp[i] = data;
                }
                data = '0';
                ser.read(&data,1);
            }
            else{
                ROS_INFO(
                "[WARNING]WRONG DATA 1111[WARNING]\n"
                "[WARNING]WRONG DATA 1111[WARNING]\n"
                "[WARNING]WRONG DATA 1111[WARNING]\n"
                "[WARNING]WRONG DATA 1111[WARNING]\n"
                );
            }
        }
        else{
        ROS_INFO("[WARNING]WRONG DATA 1111[WARNING]");
        }
        update_state(pkg_tmp);
    }
    
}

void data_printing(std::string &buffer){
    std::string pkg_tmp;
    const int n = buffer.length();
    int i=0;
    std::cout << "data: ";
    for (i=0;i<n;++i){
        printf(" %x",(unsigned char)buffer[i]);
    }
    // std::cout<<"end;" <<std::endl;
    std::cout<<std::endl;
}

int main (int argc, char** argv) 
{ 
    //初始化节点 T265_parse
    ros::init(argc, argv, "px4flow_node_T265"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
    //订阅主题，并配置回调函数 
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
    //发布主题 
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 

    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        // ser.setBaudrate(57600); 
        ser.setBaudrate(115200);
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
    // ros::Rate loop_rate(150); 
    while(ros::ok()) 
    { 

        if(ser.available()){ 
            // ROS_INFO_STREAM("Reading from serial port\n"); 
            // std::string result; 
            // result = ser.read(ser.available()); 
            // // ROS_INFO_STREAM("Read: " << result.data); 
            // std::cout << "data:"<<result<<std::endl;
            // read_pub.publish(result); 
            // std::string buffer;
            // unsigned char data;
            // ser.read(&data,1);
            // printf("%x\n",data);
            data_decoding();
            // buffer = ser.read(ser.available());
            // data_printing(buffer);
            // data_decoding(buffer);
            // std::cout << "data:"<<buffer<<std::endl;
        } 
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        // ros::spinOnce(); 
        // loop_rate.sleep(); 
    } 
}
