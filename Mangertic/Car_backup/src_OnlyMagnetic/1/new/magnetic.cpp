#include "std_msgs/String.h"
#include "magnetic.hpp"

using namespace std;

int fd;//设备描述符

//uint8_t pubbuf[8] = {0x01,0x03,0x00,0x20,0x00,0x08,0x45,0xC6};
uint8_t pubbuf[6] = {0x01,0x03,0x00,0x20,0x00,0x08};
int subbuf[8];
int turn_flag = 0;//转弯标志位

string str="string";

char *ptr;
long ret = 0;

const int MAX = 100;
int DecArr[MAX] = { 0 };

string list1;
string list2;
string list3;
string list4;

/***********************************UDP**************************************/
//创建UDP套接字
int UDPsockfd;
int UDPret;//判断UDP套接字返回值

//对方地址
struct sockaddr_in srvaddr;
socklen_t len;

char buf[50];
int i;
struct sockaddr_in cliaddr; //谁给我寄信，这个cliaddr就存放着谁的地址



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
        //usleep(50000);
        read(fd, subbuf, sizeof(subbuf));
        dec2hex(subbuf[1]);
    }
}

void redUDP()
{
    cout << "这里是UDP多线程" << endl;
    //不断接收对方给自己发送过来的信息

    while(1)
    {
        cout << "UDP收到了" << endl;
        bzero(i,sizeof(i));
        UDPret = recvfrom(UDPsockfd,i,sizeof(i),0,(struct sockaddr *)&cliaddr,&len);
        if(UDPret != -1)
        {
            printf("%s:%s",(char *)inet_ntoa(cliaddr.sin_addr),i);
        }

    }

}

void fun(int tn)
{
    int  t0 = time(NULL),last = t0,now = t0;
    tn += t0;
    cout << "tn-now1: " << (tn-now) << endl;
    while((now = time(NULL))<tn)
    if(last!=now)
    {
        cout << "左转" << endl;
        pub_msg.linear.x = 0;
        pub_msg.angular.z = 0.22;
        Sub2XQROS.publish(pub_msg);
        pub_msg.angular.z = 0;
        pub_msg.linear.x = 0;

        last = now;
    }
}

void policy()
{
    if(ret>128&&ret<8192)
    {
        if((ret>=64&&ret<128) /*||(ret>=4096&&ret<8192)*/)
        {
            cout << "直行(右)" << endl;
            pub_msg.linear.x = 0.15;
            pub_msg.angular.z = -0.10;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }
        else if(/*(ret>=64&&ret<128) ||*/(ret>=4096&&ret<8192))
        {
            cout << "直行(左)" << endl;
            pub_msg.linear.x = 0.15;
            pub_msg.angular.z = 0.10;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }


        else if((ret>=128&&ret<256) /*|| (ret>=1024&&ret<4096)*/)
        {
            cout << "直行(左)" << endl;
            pub_msg.linear.x = 0.19;
            pub_msg.angular.z = 0.06;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;

            //turn_flag = 0;
        }

        else if(/*(ret>=128&&ret<256) ||*/ (ret>=1024&&ret<4096))
        {
            cout << "直行(右)" << endl;
            pub_msg.linear.x = 0.19;
            pub_msg.angular.z = -0.06;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;

            //turn_flag = 0;
        }

        else if((ret>=256&&ret<512) /*|| (ret>=512&&ret<1024)*/)//(ret>=64&&ret<127) || (ret>=8192&&ret<16384)
        {
            cout << "直行(右)" << endl;
            pub_msg.linear.x = 0.23;
            pub_msg.angular.z = 0.02;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;

            //turn_flag = 0;
        }
        else if(/*(ret>=256&&ret<512) ||*/ (ret>=512&&ret<1024))//(ret>=64&&ret<127) || (ret>=8192&&ret<16384)
        {
            cout << "直行" << endl;
            pub_msg.linear.x = 0.23;
            pub_msg.angular.z = -0.02;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;

            //turn_flag = 0;
        }

        /*else if((ret>=16&&ret<63) || (ret>=16384&&ret<32768))
        {
            cout << "直行" << endl;
            pub_msg.linear.x = 0.15;
            pub_msg.angular.z = 0;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;

            //turn_flag = 0;
        }*/



        //ros::spinOnce();
    }

    else if(ret>=8192)//et>=32768
    {
        cout << "左转" << endl;
        pub_msg.linear.x = 0.08;
        pub_msg.angular.z = 0.35;
        Sub2XQROS.publish(pub_msg);
        pub_msg.angular.z = 0;
        pub_msg.linear.x = 0;

        turn_flag = 1;
        //ros::spinOnce();

    }

    else if(ret<=128&&ret!=0)
    {
        cout << "右转" << endl;
        cout << __LINE__ << endl;
        pub_msg.linear.x = 0.08;
        pub_msg.angular.z = -0.35;
        Sub2XQROS.publish(pub_msg);
        pub_msg.angular.z = 0;
        pub_msg.linear.x = 0;

        turn_flag = 1;
    }
    else if(ret == 0)
    {
        pub_msg.linear.x = 0;
        pub_msg.angular.z = 0;
        Sub2XQROS.publish(pub_msg);
    }
}

void UDP_init()
{
    /******************************************************************UDP设置******************************************************/

    //1.UDP套接字
    //int sockfd;
    UDPsockfd = socket(AF_INET,SOCK_DGRAM,0);

    //2. 绑定IP地址，端口号到套接字上
    struct sockaddr_in srvaddr;
    socklen_t len = sizeof(srvaddr);
    bzero(&srvaddr,len);

    srvaddr.sin_family = AF_INET;
    srvaddr.sin_port = htons(/*atoi(argv[1])*/50002);
    srvaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(UDPsockfd,(struct sockaddr *)&srvaddr,len);
}

int main(int argc,char *argv[])
{
    uint16_t crc = 0;

    

    //*********************************************************串口设置******************************************
	
	
    struct termios myios;
    bzero(&myios,sizeof(myios));
    fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY );//| O_NDELAY

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

    ros::Rate loop_rate(20);

	

    //ros::Time::init();
    // 设置循环的频率
    //ros::Rate loop_rate(1);
	// 创建节点句柄
    //ros::NodeHandle n;

    // 创建一个Publisher，发布名为BlueTooth_info的topic，消息类型为std_msgs::String(字符串)，队列长度100
    Sub2XQROS = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //std_msgs::String

    // 创建一个Subscribe，订阅名为ROS_info的topic，队列长度100
    //ROS2UARTinfo  = n.subscribe("ROS2UART_info",100,ROS2UARTcallBack);//订阅上位机通过ROS_info话题发布的信息

    // 设置循环的频率
    

    /*cout << subbuf << endl;
	int c;

	cout << "发出信息：" << endl;

	cin >> c;

	if(c == 1)
	{
		write(fd,subbuf,8);
		cout << "OK" << endl;
	}*/
    // pub_msg.linear.x = 0;
    // pub_msg.angular.z = 0;

    //初始化UDP
    UDP_init();

    std::thread thread(redSerial);
    std::thread UDPthread(redUDP);


    while(n.ok())
    {

        cout << __LINE__ << endl;

        //read(fd, subbuf, sizeof(subbuf));

        //dec2hex(subbuf[1]);
        cout << "////////////////////////////////////////////////////////////////////////" << endl;

        /*crc = Get_Modbus_Crc_16(pubbuf,6);
        printf("最终结果为：%X",crc);*/



        //cout << subbuf[0] << endl;
        //cout << hex << subbuf[1] << endl;
        //str = to_string(subbuf[1]);
        //cout << str[0] << endl;
        //cout << str[1] << endl;


        /*cout << str[2] << endl;
        cout << str[3] << endl;
        cout << str[4] << endl;
        cout << str[5] << endl;
        cout << str[6] << endl;
        cout << str[7] << endl;*/
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

        string strtest ="0x"+list1.append(list2).append(list3).append(list4);

        cout << strtest << endl;
        cout << strtest[0] << endl;
        cout << strtest[1] << endl;
        cout << strtest[2] << endl;
        cout << strtest[3] << endl;
        cout << strtest[4] << endl;
        cout << strtest[5] << endl;

        char hex2decbuf[6] = {0};
        hex2decbuf[0] = strtest[0];
        hex2decbuf[1] = strtest[1];
        hex2decbuf[2] = strtest[2];
        hex2decbuf[3] = strtest[3];
        hex2decbuf[4] = strtest[4];
        hex2decbuf[5] = strtest[5];

        ret = strtol(hex2decbuf,&ptr,0);
        cout << "ret=" << ret << endl;

        //fun(2);

        if(i==1)
        {
            policy();
        }


        loop_rate.sleep();
        ros::spinOnce();

        }

    close(fd);
    return 0;

}
