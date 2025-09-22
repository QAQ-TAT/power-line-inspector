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

    
	/*// 创建节点句柄
    ros::NodeHandle n;
    
    // 创建一个Publisher
    //ros::Publisher UART2ROSinfo;*/

  	// 创建一个Subscribe
    ros::Subscriber XQROS2UDP;

    //ros::Rate loop_rate(1);

    std_msgs::String pub_msg, sub_msg;
	std::stringstream ss;

    /**************************************UDP*************************/

    //1.创建udp套接字
    int sockfd;

    //2. 准备对方的地址
    struct sockaddr_in srvaddr;
    socklen_t len;