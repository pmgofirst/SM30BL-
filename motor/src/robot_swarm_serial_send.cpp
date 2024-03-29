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
#include <math.h>
#include <geometry_msgs/Point32.h>


// [[[need to install serial pkg first!!!!!]]] sudo apt-get install ros-kinetic-serial
// 解析时按照对应解析
#define SCALE_F 10000.0f
#define pi 3.14159265f

#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )//0-7wei
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )//8-15
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )//16-23
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )//24-31
//minpeng add
#define  BYTE4(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 4)  )
#define  BYTE5(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 5) )
#define  BYTE6(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 6) )
#define  BYTE7(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 7) )
#define  BYTE8(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 8)  )
#define  BYTE9(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 9) )
#define  BYTE10(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 10) )
#define  BYTE11(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 11) )
#define  BYTE12(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 12) )

serial::Serial ser; //声明串口对象 
int _pos[3] = {0,0,0};
int _angle[3] = {0,0,0};
int _check_sum = 0;
float float_angle[3];
float float_pos[3];
u_char begin[] = {0xfe,0x08,0x90,0x91,0x03,0x00};
u_char test_data[] = {0x23,0x23,0x23,0x23};
u_char end[] = {0xff};

float control_commands[3] = {0.0, 0.0, 0.0};
u_char sum_check={0x00};
u_char sum_check_ve={0xff};
u_char sum_check_pl={0xff};
u_char hex_low[] = {0x00};
u_char hex_high[] = {0x00};
u_char V_hex_low[] = {0x00};
u_char V_hex_high[] = {0x00};
u_char placeonecommand[] = {0xff,0xff,0x01,0x09,0x03,0x2a,0x00,0x08,0x00,0x00,0xe8,0x00,0xd5};
u_char placetwocommand[] = {0xff,0xff,0x01,0x09,0x03,0x2a,0x00,0x00,0x00,0x00,0xe8,0x00,0xdd};
u_char readtheplace[] = {0xff,0xff,0x01,0x04,0x02,0x38,0x02,0xbe};
u_char readthevelocity[] = {0xff,0xff,0x01,0x04,0x02,0x3a,0x02,0xbc};
int len = 0;
int lenone=0;
geometry_msgs::Point32 twomsg;
//minpeng 
void serial_write_int(int data){
    u_int8_t buffer[] = {BYTE0(data),BYTE1(data),BYTE2(data),BYTE3(data),BYTE4(data),BYTE5(data),BYTE6(data),BYTE7(data),
    BYTE8(data),BYTE9(data),BYTE10(data),BYTE11(data),BYTE12(data)};
    ser.write(buffer,13);
}

// 发送程序
//minpeng
void transfer_p_rad_16(float position_command,float velocity_command, u_char &position_data_low,u_char &position_data_high,u_char &Ve_low)
{

	float decimal_f = position_command*2048/pi;
	int decimal_nmber = decimal_f;
	position_data_high = (decimal_nmber/256)&0xff;
	position_data_low = decimal_nmber&0xff;

	float Ve_f_d = velocity_command/0.732;
	int Ve_f_int = Ve_f_d;
	Ve_low = Ve_f_int&0xff;
	//position_data_low = decimal_low; 
	//position_data_high=decimal_high;
	//ROS_INFO("tp%f_%d_%X_%X_%X",decimal_f,decimal_nmber,decimal_nmber,position_data_high,position_data_low);
	//int decimal_high = decimal_nmber/256;
	//int decimal_low = decimal_nmber%256;
	//ltoa(decimal_high,position_data_high);
	//ltoa(decimal_low,position_data_low);
}
 
void serial_write_data()
{	
	transfer_p_rad_16(control_commands[0],control_commands[1],hex_low[0],hex_high[0],V_hex_low[0]);
    placeonecommand[6] = hex_low[0] ;
    placeonecommand[7] = hex_high[0];
    placeonecommand[10]= V_hex_low[0];
    sum_check=~(placeonecommand[2]+placeonecommand[3]+placeonecommand[4]+placeonecommand[5]+placeonecommand[6]
    	+placeonecommand[7]+placeonecommand[8]+placeonecommand[9]+placeonecommand[10]+placeonecommand[11]);
    placeonecommand[12]=sum_check;
    //ROS_INFO("testpalcecommand%X_%X",placeonecommand[7],placeonecommand[8]);
    /*ROS_INFO("check%X_%X_%X_%X_%X_%X_%X_%X_%X_%X_%X_%X_%X_%X",placeonecommand[0],placeonecommand[1],placeonecommand[2],placeonecommand[3],
    	placeonecommand[4],placeonecommand[5],placeonecommand[6],placeonecommand[7],placeonecommand[8],
    	placeonecommand[9],placeonecommand[10],placeonecommand[11],placeonecommand[12],sum_check);*/
    ser.write(placeonecommand,13);
    ROS_INFO("velocitytxd=%X",placeonecommand[10]);
    ser.flushInput ();
    //ros::Duration(2);
 /*   ser.write(readtheplace,8);
    //ros::Duration(0.1);
    ser.write(readthevelocity,8);
    //ros::Duration(0.1);
    ser.write(placetwocommand,13);
    //ros::Duration(2);
    ser.write(readtheplace,8);
    //ros::Duration(0.1);
    ser.write(readthevelocity,8);
    //ros::Duration(0.1);   */
}
void serial_write_dataone()
{ser.write(placetwocommand,13);}
void serial_write_datatwo()
{ser.write(readthevelocity,8);}
/*
int c = 1, d = 2;
aaa(c, d);

void aaa(int a, int& b)
{
	a = 3;
	b = 4;
}
cout << c << d << endl;
c = 1;
d = 4;

u_char low, high;
transfer_p_rad_16(control_commands[0], low, high);
placeonecommand[7] = low;
*/


// 接受程序
void velocity_serial_data(){
	//len = frm->data.size(); get the size of the available data
	ser.flushInput ();
	//ros::Duration(0.001).sleep();
	ser.write(readthevelocity,8);
	len = ser.getBytesize();
    if(len!=0){   //ser.available()
    	static u_int8_t buffer[16];
        ser.read(buffer,len);//从串口读取数据并保存
        //ROS_INFO("the velocity%d_%d",buffer[6],buffer[5]);
         for(int i = 0;i<len;i++)
        {ROS_INFO("%X",buffer[i]);}
        sum_check_ve = ~(buffer[2]+buffer[3]+buffer[4]+buffer[5]+buffer[6]);//check the message wo moren yichu qudiwei
        //ROS_INFO("Velocity=====%X_%X_%X_%X_%X_%X_%X_%X_%X",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],sum_check_ve);
        //data = sum_check&comparedata;
        //if(sum_check_ve == buffer[7]){
        	//data=buffer[6]*512+
        	//ROS_INFO("%X_%X_%X_%X_%X_%X_%X_%X",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],
             //   buffer[6],buffer[7]);
    	if(sum_check_ve == buffer[7]){
        	int velocity_read_low = buffer[5];
        	float velocity_read = 0.0;
        	if(buffer[6]==0x00){
        		velocity_read = velocity_read_low *0.732f;
        		ROS_INFO("Velocity=%frpm",velocity_read);}
        	else if(buffer[6]==0x80){
        		velocity_read = -velocity_read_low *0.732f;
        		ROS_INFO("Velocity=%frpm",velocity_read);
        	}
        	else{ROS_INFO("VelocityInfoError");}
        	twomsg.y = velocity_read;
    	}
    	
    }
    //ROS_INFO("velocity read finished");
    //ROS_INFO("velocity%X_%X_%X_%X_%X_%X_%X_%X",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],
    //            buffer[6],buffer[7]);
    ser.flushInput (); // clean the last data
    ros::Duration(0.001).sleep();
}

void place_serial_data(){
	//len = frm->data.size(); get the size of the available data
	ser.flushInput ();
	//ros::Duration(0.001).sleep();
	ser.write(readtheplace,8);
	lenone = ser.getBytesize();
    if(lenone!=0){   //ser.available()
        static u_int8_t bufferone[16];
        ser.read(bufferone,lenone);//从串口读取数据并保存
        //ROS_INFO("the place%d_%d",bufferone[6],bufferone[5]);
        //for(int i = 0;i<len;i++)
        //	{ROS_INFO("%X",bufferone[i]);}
        sum_check_pl=~(bufferone[2]+bufferone[3]+bufferone[4]+bufferone[5]+bufferone[6]);//check the message wo moren yichu qudiwei
       for(int j=0;j<lenone;j++)
       	{ROS_INFO("%X",bufferone[j]);}
        //data = sum_check&comparedata;
        if(sum_check_pl == bufferone[7]){
        	int place_read_low = bufferone[5];
        	int place_read_highbe = bufferone[6];
        	int place_read_high = place_read_highbe*256;
        	float place_read = (place_read_low + place_read_high)*pi/2048;
        	ROS_INFO("the place=%f",place_read);
        	twomsg.x = place_read;
        }
        	//data=buffer[6]*512+
        //	ROS_INFO("%X_%X_%X_%X_%X_%X_%X_%X",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],
        //        buffer[6],buffer[7]);
        //ROS_INFO("place=%X_%X",buffer[6],buffer[5]);
        //ROS_INFO()
    	//}
    	//ROS_INFO("place%X_%X_%X_%X_%X_%X_%X_%X",bufferone[0],bufferone[1],bufferone[2],bufferone[3],bufferone[4],bufferone[5],
         //      bufferone[6],bufferone[7]);
    }
    //ROS_INFO("place read finished");
    ser.flushInput (); // clean the last data
    ros::Duration(0.001).sleep();
}


void commandsCallback(const geometry_msgs::Point32& msg)
{
	control_commands[0] = msg.x;
	control_commands[1] = msg.y;
	control_commands[2] = msg.z;
	ROS_INFO("test%f",control_commands[0]);

}

int main (int argc, char** argv) 
{   //指定循环的频率 
    //初始化节点 

    ros::init(argc, argv, "robot_swarm_serial_send"); 

    //声明节点句柄 
    ros::NodeHandle nh; 
    ros::Subscriber sub = nh.subscribe("/gimbal_commands", 1, commandsCallback);
    // publisher
    ros::Publisher po_ve_pub = nh.advertise<geometry_msgs::Point32>("place_velocity_info", 100);
    try 
    { 
    //设置串口属性，并打开串口 
        //ser.setPort("/dev/ttyS0"); 
	ser.setPort("/dev/ttyUSB0"); 
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
    // serial_write_data();
    //minpeng

    ros::Rate loop_rate(40); 
    while(ros::ok()) 
    {	
    	
        serial_write_data();
        ros::Duration(0.01).sleep();
        //ser.flushInput();
        // write_callback_test();
        // loop_rate.sleep(); 
        //serial_write_datatwo(); 
        //velocity_serial_data();
        //ros::Duration(0.001).sleep(); 
        place_serial_data();
        //twomsg.x = place_read;
        ros::Duration(0.001).sleep();
        velocity_serial_data();
        //twomsg.y = velocity_read;
        //twomsg.z = 0.0;
        ros::Duration(0.001).sleep();
        po_ve_pub.publish(twomsg);
        // serial_write_dataone();
        //ros::Duration(0.1).sleep();
        //int kkk=1000; int jjj=kkk&0xff00; 
        //float kkk=1000.0;int jjj =kkk;
        //ROS_INFO("ceshidetohex%i",jjj);
        loop_rate.sleep();
        ros::spinOnce();
    } 
}
