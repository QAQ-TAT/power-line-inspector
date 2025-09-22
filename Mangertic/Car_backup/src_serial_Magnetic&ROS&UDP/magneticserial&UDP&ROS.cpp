#include "../include/Sub2ROS.hpp"

using namespace std;

int fd;//磁力传感器串口设备描述符
char XQROS2UDPbuf[8]= {0};

uint16_t crc = 0;

//uint8_t pubbuf[8] = {0x01,0x03,0x00,0x20,0x00,0x08,0x45,0xC6};
uint8_t pubbuf[6] = {0x01,0x03,0x00,0x20,0x00,0x08};
int subbuf[8];

string str="string";

char *ptr;
long ret = 0;
char hex2decbuf[6] = {0};
string strtest;

const int MAX = 100;
int DecArr[MAX] = { 0 };

string list1;
string list2;
string list3;
string list4;



uint16_t Get_Modbus_Crc_16(uint8_t *buffer, uint16_t len)
{
    cout << "len:" << len << endl;
	uint16_t calcrc = 0XFFFF;//(1)16位CRC寄存器赋初始值
	uint8_t  temp;
	uint16_t  i = 0, j = 0;//计数
	for (i = 0; i < len; i++)//(6)除最后两位CRC位，其余每个字节数据都需要计算
	{
		temp = *buffer & 0XFF;
		buffer++;
		calcrc = calcrc ^ temp;//(2)将八位数据与CRC寄存器亦或，数据存入CRC寄存器
		for (j = 0; j < 8; j++)//(5)每字节的8位数据都需计算
		{
			if (calcrc & 0X0001)//判断即将右移出的位是不是1，如果是1则与0XA001进行异或。
			{
				calcrc = calcrc >> 1;//(3)先将数据右移一位
				calcrc = calcrc ^ 0XA001;//(4)数据与0XA001进行异或
			}
			else
			{
				calcrc = calcrc >> 1;//(3)(4)如果是0，直接移出
			}
      printf("第%d次转换为%04X\n",j+1,calcrc);
		}
    printf("第%d个字节转换结束\n",i+1);
	}
	uint8_t  CRC_L;
	uint8_t  CRC_H;
	CRC_L = calcrc & 0xFF;//CRC的低八位
	CRC_H = calcrc >> 8;//CRC的高八位
	return ((CRC_L << 8) | CRC_H);//(7)返回CRC最终值，低位在左，高位在右
}


string dec2hex(int i) //将int转成16进制字符串
{
	stringstream ioss; //定义字符串流
	//string s_temp; //存放转化后字符
	ioss << setiosflags(ios::uppercase) << hex << i; //以十六制(大写)形式输出
	//ioss << resetiosflags(ios::uppercase) << hex << i; //以十六制(小写)形式输出//取消大写的设置
	ioss >> str;

	//cout << "str " << s_temp << endl;;
	return str;
}

int hex2char(uint8_t c)
{
    return ((c >= '0') && (c <= '9')) ? int(c - '0') :
           ((c >= 'A') && (c <= 'F')) ? int(c - 'A' + 10) :
           ((c >= 'a') && (c <= 'f')) ? int(c - 'a' + 10) :
           -1;
}

int hex2dec(int aHex)
{
	long Dec = 0;
	int temp = 0;
	int count = 0;

	while (0 != aHex)//循环直至aHex的商为零
	{
	   cout << aHex % 16 << endl;//测试aHex%16取余得到得是不是一位
		temp = aHex;
		aHex = aHex / 16;//求商
		temp = temp % 16;//取余
		DecArr[count++] = temp;//余数存入数组
	}
	//
	int j = 0;
	for (int i = 0; i<count;i++ )
	{
		if (i < 1)
		{
			Dec = Dec + DecArr[i];
		}
		else
		{
		//16左移4位即16²，左移8位即16³、以此类推。
			Dec = (Dec + (DecArr[i]*(16<<j)));
			j += 4;
		}
	}

	cout << Dec << endl;

	return 0;
}

void redSerial()
{
	while(1)
	{
            usleep(200000);
            read(fd, subbuf, sizeof(subbuf));
            dec2hex(subbuf[1]);
	}
}

//收到ROS的回调函数

void XQROS2UDPcallBack(const std_msgs::Float64& sub_msg)
{

    setlocale(LC_ALL, "");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    cout << "sub_msg.data[0]" << sub_msg.data << endl;

    ROS_INFO("收到XQROS信息： %s", sub_msg.data);

    XQROS2UDPbuf[0] = sub_msg.data;

    cout << "ROS2UART_recvbuffer[0]:" << XQROS2UDPbuf[0] << endl;

}


//UDP发送 线程

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

        else if(XQROS2UDPbuf[0] == 1)
        {
            cout << "上升" << endl;

            sendto(sockfd,XQROS2UDPbuf,strlen(XQROS2UDPbuf),0,(struct sockaddr *)&srvaddr,len);
        }

        else if(XQROS2UDPbuf[0] == 2)
        {
            cout << "下降" << endl;

            sendto(sockfd,XQROS2UDPbuf,strlen(XQROS2UDPbuf),0,(struct sockaddr *)&srvaddr,len);
        }
    }

    //4.销毁信箱
    close(sockfd);
}

void Serial2ROS()
{

    while(1)
    {

        cout << "////////////////////////////////////////////////////////////////////////" << endl;


        cout << "01234567" << endl;
        cout << str[0] << endl;
        cout << str[1] << endl;
        cout << str[2] << endl;
        cout << str[3] << endl;
        cout << str[4] << endl;
        cout << str[5] << endl;
        cout << str[6] << endl;
        cout << str[7] << endl;
        cout << sizeof(str) << endl;

        if(str[7] == NULL)
        {
            str[7] = str[6];
            str[6] = str[5];
            str[5] = str[4];
            str[4] = str[3];
            cout << "++++" << endl;
        }

        cout << "6745" << endl;
        list1 = str[6];
        list2 = str[7];
        list3 = str[4];
        list4 = str[5];

        cout << "list1234" << endl;
        cout << list1 << endl;
        cout << list2 << endl;
        cout << list3 << endl;
        cout << list4 << endl;

        strtest ="0x"+list1.append(list2).append(list3).append(list4);

        cout << strtest << endl;
        cout << strtest[0] << endl;
        cout << strtest[1] << endl;
        cout << strtest[2] << endl;
        cout << strtest[3] << endl;
        cout << strtest[4] << endl;
        cout << strtest[5] << endl;



        hex2decbuf[0] = strtest[0];
        hex2decbuf[1] = strtest[1];
        hex2decbuf[2] = strtest[2];
        hex2decbuf[3] = strtest[3];
        hex2decbuf[4] = strtest[4];
        hex2decbuf[5] = strtest[5];

        ret = strtol(hex2decbuf,&ptr,0);
        cout << "ret=" << ret << endl;

        if(ret>=112&&ret<=3840)
        {
            cout << "直行" << endl;
            pub_msg.linear.x = 0.30;
            pub_msg.angular.z = 0;
            Sub2XQROS.publish(pub_msg);
        }

        else if(ret>3840)
        {
            cout << "左转" << endl;
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0.30;
            Sub2XQROS.publish(pub_msg);
        }

        else if(ret<224&&ret!=0)
        {
            cout << __LINE__ << endl;
            cout << "右转" << endl;
            cout << __LINE__ << endl;
            pub_msg.linear.x = 0;
            pub_msg.angular.z = -0.30;
            Sub2XQROS.publish(pub_msg);
            cout << __LINE__ << endl;
        }

       else if(ret == 0)
        {
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
            Sub2XQROS.publish(pub_msg);

        }
    }
}

int main(int argc,char *argv[])
{
    uint16_t crc = 0;

    

    //*********************************************************串口设置******************************************
	
	
    struct termios myios;
	bzero(&myios,sizeof(myios));
        fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);//| O_NDELAY
	if(fd == -1)
	{
		perror("打开串口失败！\n");
		return -1;
	}

	//保存当前串口的配置
	tcgetattr(fd,&myios);

	//设置串口工作在原始模式
	cfmakeraw(&myios);
	myios.c_cflag |= CLOCAL | CREAD; 
	
	//设置波特率
	cfsetispeed(&myios,B115200);
	cfsetospeed(&myios,B115200);
	
	//设置8位数据位，无奇偶校验
	myios.c_cflag &= ~CSIZE;
    myios.c_cflag |= CS8;  //CS7    CS6  CS5  
    //无奇偶校验
    myios.c_cflag &= ~PARENB;
	
	//1位停止位
	myios.c_cflag &= ~CSTOPB;
	
	//刷新缓冲区
	myios.c_cc[VTIME] = 0;
    myios.c_cc[VMIN] = 1;
    tcflush(fd,TCIOFLUSH);  //刷新输入输出缓冲区
	
	//串口设置使能
	tcsetattr(fd,TCSANOW,&myios);

	//*******************************************************ROS设置********************************************************

	// ROS节点初始化
    ros::init(argc, argv, "test");

	// 创建节点句柄
    ros::NodeHandle n;

    ros::Rate loop_rate(5);

    // 创建一个Publisher，发布名为BlueTooth_info的topic，消息类型为std_msgs::String(字符串)，队列长度100
    Sub2XQROS = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

     // 创建一个Subscribe，订阅名为XQROS2UDP_info的topic，队列长度100  回调函数ROS2UARTcallBack
    XQROS2UDP  = n.subscribe("XQROS2UDP_info",10,XQROS2UDPcallBack);//订阅xiaoqiang ROS 信息发布到UDP上



    /*******************************************************UDP设置**************************************/

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

    

/*******************************************************功能函数***********************************/

//	std::thread thread(redSerial);//读取串口信息多线程
//        std::thread UDPthread(UDPsend);  // 启动UDP发送线程
//        std::thread Serial2ROSthread(Serial2ROS);  // 启动UDP发送线程

	while(n.ok())
	{
            cout << __LINE__ << endl;
            std::thread thread(redSerial);//读取串口信息多线程
            //std::thread UDPthread(UDPsend);  // 启动UDP发送线程
            std::thread Serial2ROSthread(Serial2ROS);  // 启动串口给ROS


        loop_rate.sleep();
        //ros::spinOnce();
             ros::spin();

	}

	close(fd);
        close(sockfd);
	return 0;
}
