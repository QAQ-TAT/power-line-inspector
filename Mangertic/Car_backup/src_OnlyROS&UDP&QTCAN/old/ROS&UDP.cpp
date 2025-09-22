#include "ROS&UDP.hpp"

using namespace std;
char XQROS2UDPbuf[8]= {0};
socklen_t receive_len;


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

void UDPreceive()
{
    char buf[50];
    struct sockaddr_in cliaddr; //谁给我寄信，这个cliaddr就存放着谁的地址
    int ret;
    while(1) 
    {
        bzero(buf,sizeof(buf));
        ret = recvfrom(sockfd,buf,sizeof(buf),0,(struct sockaddr *)&cliaddr,&receive_len);
        if(ret != -1)
        {
            printf("%s:%s",(char *)inet_ntoa(cliaddr.sin_addr),buf);
        }
        
        if(strncmp(buf,"quit",4) == 0)
        {
            break;
        }

        else if(buf[0] == 0)//_ _ _ _ _ _ _
        {
            buf[1] = 0;
            buf[2] = 0;
            buf[3] = 0;
            buf[4] = 0;
            buf[5] = 0;
            buf[6] = 0;
            XQROS2UDPbuf[0] = 100;//电机暂停
            pub_msg.linear.x = 0;//线速度0
            pub_msg.angular.z = 0;//角速度0

            sendto(sockfd,XQROS2UDPbuf,strlen(XQROS2UDPbuf),0,(struct sockaddr *)&srvaddr,len);//向电机发送暂停
            UDP2XQROS.publish(pub_msg);//向XQROS /cmd_vel 发送速度
        }

        else if(buf[0] == 1)
        {
            if(buf[1] == 1)//向前
            {
                cout << "直行" << endl;
                pub_msg.linear.x = 0.30;
                pub_msg.angular.z = 0;
                UDP2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }

            else if(buf[2] == 1)
            {
                cout << "后退" << endl;
                pub_msg.linear.x = -0.30;
                pub_msg.angular.z = 0;
                UDP2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }

            else if(buf[3] == 1)
            {
                cout << "向左" << endl;
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0.3;
                UDP2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }

            else if(buf[4] == 1)
            {
                cout << "向右" << endl;
                pub_msg.linear.x = 0;
                pub_msg.angular.z = -0.3;
                UDP2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }

            else if(buf[5] == 1)
            {
                cout << "上升" << endl;
                XQROS2UDPbuf[0] = 35;//电机上升
                sendto(sockfd,XQROS2UDPbuf,strlen(XQROS2UDPbuf),0,(struct sockaddr *)&srvaddr,len);
            }

            else if(buf[5] == 1)
            {
                cout << "下降" << endl;
                XQROS2UDPbuf[0] = 125;//电机下降
                sendto(sockfd,XQROS2UDPbuf,strlen(XQROS2UDPbuf),0,(struct sockaddr *)&srvaddr,len);
            }
        }
    }
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
    XQROS2UDP  = n.subscribe("XQROS2UDP_info",100,XQROS2UDPcallBack);//订阅xiaoqiang ROS 信息发布到UDP上 电机运行
    UDP2XQROS = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//前后左右

    // 设置循环的频率
    ros::Rate loop_rate(20);



    /******************************************************************UDP设置******************************************************/
    /************************************************发送端配置***********************************************/
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

    /***********************************************接收端配置*************************************************/

    //2. 绑定IP地址，端口号到套接字上
    struct sockaddr_in receiver;
    socklen_t len_receiver = sizeof(receiver);
    receive_len = len_receiver;
    bzero(&receiver,receive_len);
    
    receiver.sin_family = AF_INET;
    receiver.sin_port = htons(/*atoi(argv[3])*/50001);//接收端口号(自己端口号)
    receiver.sin_addr.s_addr = htonl(INADDR_ANY);
    
    bind(sockfd,(struct sockaddr *)&receiver,len_receiver);


	/*******************************************************************功能函数************************************/
    //std::thread UDPthread(UDPsend);  // 启动UDP线程

    // pthread_t tid;
    // pthread_create(&tid,NULL,routine,(void *)&sockfd);

	while(n.ok())
	{

        std::thread thread(mass);  // 启动UDP线程
        std::thread UDPsendThread(UDPsend);  // 启动UDP发信线程
        std::thread UDPreceiveThread(UDPreceive);  // 启动UDP收信线程


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
