#include "ROS&UDP.hpp"

using namespace std;
char XQROS2UDPbuf[8]= {0};


void XQROS2UDPcallBack(const std_msgs::Float64& sub_msg)
{

    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    cout << "sub_msg.data[0]" << sub_msg.data << endl;

    ROS_INFO("收到XQROS信息： %s", sub_msg.data);

    XQROS2UDPbuf[0] = sub_msg.data;

    cout << "ROS2UART_recvbuffer[0]:" << XQROS2UDPbuf[0] << endl;

}

void UDPsend()
{
    while(1)
    {
        usleep(200000);

        if(XQROS2UDPbuf[0] == 0)
        {
            cout << "暂停" << endl;

            sendto(sockfd,XQROS2UDPbuf,strlen(XQROS2UDPbuf),0,(struct sockaddr *)&srvaddr,len);
        }

        else if(XQROS2UDPbuf[0] > 0)
        {
            cout << "上升" << endl;

            sendto(sockfd,XQROS2UDPbuf,strlen(XQROS2UDPbuf),0,(struct sockaddr *)&srvaddr,len);
        }

        else if(XQROS2UDPbuf[0] >0 && XQROS2UDPbuf[0] < 1)
        {
            cout << "下降" << endl;

            sendto(sockfd,XQROS2UDPbuf,strlen(XQROS2UDPbuf),0,(struct sockaddr *)&srvaddr,len);
        }

         else if(XQROS2UDPbuf[0] < 0)
        {
            cout << "<<<<<<<0"
        }
    }

    //4.销毁信箱
    close(sockfd);
}

void mass()
{

    cout << "mass" << endl;
    char buf[50];
    int ret;

    while(1)
    {
        bzero(buf,sizeof(buf));
        fgets(buf,50,stdin);
        sendto(sockfd,buf,strlen(buf),0,(struct sockaddr *)&srvaddr,len);
    }

    close(sockfd);

}

int main(int argc,char *argv[])
{

	/**************************************************ROS设置********************************************************/

	// ROS节点初始化
    ros::init(argc, argv, "XQ2UDP");

	// 创建节点句柄
    ros::NodeHandle n;

    //ros::Time::init();
    // 设置循环的频率
    //ros::Rate loop_rate(1);
	// 创建节点句柄
    //ros::NodeHandle n;

    // 创建一个Publisher，发布名为BlueTooth_info的topic，消息类型为std_msgs::String(字符串)，队列长度100
    //UART2ROSinfo = n.advertise<std_msgs::Float64MultiArray>("UART2ROS_info", 100);//通过BlueTooth_info发布串口信息给上位机
    //std_msgs::String

    // 创建一个Subscribe，订阅名为ROS_info的topic，队列长度100  回调函数ROS2UARTcallBack
    XQROS2UDP  = n.subscribe("XQROS2UDP_info",100,XQROS2UDPcallBack);//订阅xiaoqiang ROS 信息发布到UDP上

    // 设置循环的频率
    ros::Rate loop_rate(20);



    /******************************************************************UDP设置******************************************************/

	//1.UDP套接字
	//int sockfd;
	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	
	//2. 准备对方的地址
	//struct sockaddr_in srvaddr;
	len = sizeof(srvaddr);
	bzero(&srvaddr,len);
	
	srvaddr.sin_family = AF_INET;
	srvaddr.sin_port = htons(/*atoi(argv[2])*/50001);//端口号
	inet_pton(AF_INET,/*argv[1]*/"192.168.3.186",&srvaddr.sin_addr);//IP地址




	/*******************************************************************功能函数************************************/
    //std::thread UDPthread(UDPsend);  // 启动UDP线程
	while(n.ok())
	{

            std::thread thread(mass);  // 启动UDP线程
            std::thread UDPthread(UDPsend);  // 启动UDP线程


            //std::thread thread_UARThread(UARThread);//启动UART线程

            cout << "线程" << endl;

            ros::spin();


            /*loop_rate.sleep();
            ros::spinOnce();*/

	}

    //4.销毁信箱
    close(sockfd);

return 0;

}
