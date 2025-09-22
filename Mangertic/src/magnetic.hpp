#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <vector>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <thread>//多线程

#include <stdio.h>
#include <strings.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <dirent.h>
#include <sys/wait.h>
#include <signal.h>
#include <linux/input.h>
#include <time.h>
#include <stdbool.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>
#include <cstdlib>
//#include "rfid.h"
#include "ControlCAN.hpp"
using namespace std;

/*
	// 创建节点句柄
    ros::NodeHandle n;*/
    
    // 创建一个Publisher
    ros::Publisher Sub2XQROS;

    // 创建一个XQROS2UDP Subscribe
    ros::Subscriber XQROS2UDP;

    geometry_msgs::Twist pub_msg, sub_msg;
	std::stringstream ss;


/***********************************UDP**************************************/
//创建UDP套接字
    int UDPsockfd;
    int UDPret;//判断UDP套接字返回值


    struct sockaddr_in cliaddr; //谁给我寄信，这个cliaddr就存放着谁的地址
    struct sockaddr_in srvaddr;

    socklen_t len_UDP;

    char buf_receiveUDP[5];//接收UDP信息 暂时接收Auto、Back、↑↓← ☞
    //uint8_t pubbuf[8] = {0x01,0x03,0x00,0x20,0x00,0x08,0x45,0xC6};
    uint8_t pubbuf[6] = {0x01,0x03,0x00,0x20,0x00,0x08};
    int subbuf[8];
    int turn_flag = 0;//转弯标志位


/***************************************磁力传感器串口****************/
    int usbfd_magnetic;//磁传感器设备描述符 

    char PolicyFlag[50];
    // char AutoForwardFlag[50];//自动寻迹标志位
    // char AutoBackFlag[50];//撤退标志位

    //int AutoForwardStopFlag = 1;//用于Auto自动停止不然会一直某状态运行
    //int AutoBackStopFlag = 1;//用于Auto自动停止不然会一直某状态运行

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓old↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
    char *ptr;//过滤字符
    long ret_magnetic = 0;//磁力传感器接收到的数值

    string str="string";//处理串口发来的信息
    string list1;
    string list2;
    string list3;
    string list4;

    const int MAX = 100;
    int DecArr[MAX] = { 0 };
/*↑↑↑↑↑↑↑↑↑↑↑↑↓↑↑↑↑old↑↑↑↑↑↑↑↓↓↑↑↑↑↑↑↑↑↑*/

int magneticMax;//magnetic 16通道中最大值
int magneticRank;//第几位是最大
int tmp;//用于转换接收到的数值排序
unsigned char buf_ReadMagnetic[22];//接收到串口信息
unsigned char buf_WriteMagnetic[8] = {0x01,0x03,0x00,0x20,0x00,0x08,0x45,0xC6};//读取串口信息的前置条件


/***************************************RFID串口********************/
    int fd_rfid;//RFID设备描述符 
    int ret_rfid=1,rec=1, i,len=0;//int ret_rfid;
    struct timeval timeout_RFID;
    volatile unsigned int cardid;
    char cardidstr[100];
    int init_rfidSerial();

/***************************************USBCAN串口********************/
    int fd_USBCAN = 0;
    int recvlen_can;//can收到的帧数 一帧包括 .data[1]~data[x] 帧id 等等信息
    VCI_CAN_OBJ recv_msg[2]; //can接收的信息
    VCI_CAN_OBJ send_MotorForce[2];

/***************************************USBIIC串口********************/
    int fd_usbiic;//USBIIC设备描述符 
    unsigned char WBuf[8], RBuf[128];// WBuf:向设备写入的数据用来配置ttyACM0设备 RBuf：读取设备信息 第一个数组RBuf[0] 是电位器的数值 然后吧RBuf[0]的值传递给 ActualPosition（实际位置）
    struct timeval timeout_USBIIC;

/**************************************processData处理数据**************/
    string strtest;
    char hex2decbuf[6] = {0};
 /*************************************自动充电**************************/
    int flag_lowbattery = 0;// 放弃该标志位表示高低电量
    int flag_batteryHightOrLow = 0x48;//0x48 H高电量      0x4C L 低电量
    int base = 0;

 /*************************************标志位（接收上位机的一些标志位和上下左右升降）**************************/

    int AutoFlag;//自动循迹标志位
    int AutoForwardOrBackFlag;//自动寻迹前进标志位
    int DirectionFlag;//遥控标志位

    double ActualPosition;//杆子实际位置 杆的位置ActualPosition = RBuf[0] 后 ActualPosition = ((double)180/132)*ActualPosition
    double ExpectPosition = 20;//期望位置 即上位机发来的期望位置0~180