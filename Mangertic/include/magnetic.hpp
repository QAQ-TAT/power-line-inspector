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
//#include "rfid.h"

#include "ControlCAN.hpp"
#include "libusb.hpp"

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

    char PolicyFlag[50];
    char AutoFlag[50];

    struct sockaddr_in cliaddr; //谁给我寄信，这个cliaddr就存放着谁的地址
    struct sockaddr_in srvaddr;

    socklen_t len_UDP;

/***************************************磁力传感器串口****************/
    int fd_magnetic;//磁传感器设备描述符 

/***************************************RFID串口********************/
    int fd_rfid;//RFID设备描述符 
    struct timeval timeout;
    volatile unsigned int cardid;
    int init_rfidSerial();

/***************************************USBCAN串口********************/
int ret_USBCAN = 0;
