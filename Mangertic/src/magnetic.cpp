#include "magnetic.hpp"


/**********************************************************接受信息用不到CRC校验********************************************/
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

int hex2char(uint8_t c)
{
    return ((c >= '0') && (c <= '9')) ? int(c - '0') :
           ((c >= 'A') && (c <= 'F')) ? int(c - 'A' + 10) :
           ((c >= 'a') && (c <= 'f')) ? int(c - 'a' + 10) :
           -1;
}

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓processData↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
void processData_magnetic_old()
{
    while(1)
    {
        // cout << "////////////////////////////////////////////////////////////////////////" << endl;

        // cout << "01234567" << endl;
        // cout << str[0] << endl;
        // cout << str[1] << endl;
        // cout << str[2] << endl;
        // cout << str[3] << endl;
        // cout << str[4] << endl;
        // cout << str[5] << endl;
        // cout << str[6] << endl;
        // cout << str[7] << endl;
        // cout << sizeof(str) << endl;

        if(str[7] == NULL)
        {
            str[7] = str[6];
            str[6] = str[5];
            str[5] = str[4];
            str[4] = str[3];
            cout << "++++" << endl;
        }

        // cout << "6745" << endl;
        list1 = str[6];
        list2 = str[7];
        list3 = str[4];
        list4 = str[5];

        // cout << "list1234" << endl;
        // cout << list1 << endl;
        // cout << list2 << endl;
        // cout << list3 << endl;
        // cout << list4 << endl;

        strtest ="0x"+list1.append(list2).append(list3).append(list4);

        cout << " " << strtest << endl;
        cout << " " << strtest[0];
        cout << " " << strtest[1];
        cout << " " << strtest[2];
        cout << " " << strtest[3];
        cout << " " << strtest[4];
        cout << " " << strtest[5] << endl;

        //char hex2decbuf[6] = {0}; //->magnetic.hpp
        hex2decbuf[0] = strtest[0];
        hex2decbuf[1] = strtest[1];
        hex2decbuf[2] = strtest[2];
        hex2decbuf[3] = strtest[3];
        hex2decbuf[4] = strtest[4];
        hex2decbuf[5] = strtest[5];

        usleep(50000);

    }
}
void processData_usbCanMotor(int motorforce)
{
    recvlen_can = 1;
    send_MotorForce[0].ID = 0x00000200;
    send_MotorForce[0].SendType = 0;
    send_MotorForce[0].RemoteFlag = 0;
    send_MotorForce[0].ExternFlag = 0;
    send_MotorForce[0].DataLen = 8;
    send_MotorForce[0].Data[0] =(motorforce>>8)&0xFF; //0C 2A 0C 2A 0C 2A 0C 2A  printf("%02x \n", recv_msg[0].Data[0]);
    send_MotorForce[0].Data[1] =motorforce&0xFF;
    send_MotorForce[0].Data[2] =(motorforce>>8)&0xFF;
    send_MotorForce[0].Data[3] =motorforce&0xFF;
    send_MotorForce[0].Data[4] =(motorforce>>8)&0xFF;
    send_MotorForce[0].Data[5] =motorforce&0xFF;
    send_MotorForce[0].Data[6] =(motorforce>>8)&0xFF;
    send_MotorForce[0].Data[7] =motorforce&0xFF;

    for (int j=0; j< 8; j++) 
    {
        printf("%02x ", send_MotorForce[i].Data[j]);
    }

//     for(int i=0;i<8;i+=2)
//    {
//    		send_MotorForce[0].Data[i] = (motorforce>>8)&0xFF;
//    }
//     for(int i=1;i<8;i+=2)
//    {
//    		send_MotorForce[0].Data[i] = motorforce&0xFF;
//    }

   VCI_Transmit (4, 0, 0, send_MotorForce, recvlen_can);

}
/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑processData↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓RFID↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
/*计算校验和*/
unsigned char CalBCC(unsigned char *i, int ln)
{
	unsigned char j, tempq = 0;
	i += 4;
	for(j = 0; j<(ln-4); j++)
	{
		tempq ^= *i;
		i++;
	}
	
	return tempq; 
}

int PiccRequest(int fd_rfid)
{
	unsigned char WBuf[128], RBuf[128];
	int  ret_rfid, i,len=0;
	fd_set rdfd;/////f*d_set rdf*d;*******************************************
	
	memset(WBuf, 0, 128);  //数组清空 向WBuf中写入128个0
	memset(RBuf,0,128);
	
	
	WBuf[0] = 0xAA;	
	WBuf[1] = 0x55;	
	WBuf[2] = 0x06;	
	WBuf[3] = 0x00;	
	WBuf[4] = 0x00;	
	WBuf[5] = 0x00;		
	WBuf[6] = 0x01;
	WBuf[7] = 0x02;
	WBuf[8] = 0x52;
	WBuf[9] = CalBCC(WBuf,9);
	
	while(1)
	{
		FD_ZERO(&rdfd);
		FD_SET(fd_rfid,&rdfd);
		tcflush (fd_rfid, TCIFLUSH);  
		
		write(fd_rfid, WBuf, 10);
		
	
		ret_rfid = select(fd_rfid + 1,&rdfd, NULL,NULL,&timeout_RFID);  //监测文件描述符的变化
		switch(ret_rfid)
		{
			case -1:
				perror("select error\n");
				break;
			case  0: //超时
				len++;   //3次请求超时后，退出该函数
				if(len == 3)
				{
					len=0;
					return -1;
				}
				
				//printf("Request timed out.\n");
				break;
			default: //说明有数据可读
				usleep(10000);  //微秒
				ret_rfid = read(fd_rfid, RBuf, 12);
				
				if(ret_rfid < 0)
				{
					printf("len = %d, %d\n", ret_rfid, errno);
					break;
				}
				
				//printf("RBuf[2]:%x\n", RBuf[2]);
				if(RBuf[8] == 0x00)	 	//应答帧状态部分为0 则请求成功
				{
					return 0;
				}
				break;
		}
		
		usleep(100000);
		
	}
	
	return -1;
}

//蜂鸣器
int buzzer(int fd_rfid)
{
	unsigned char FM[128],RBuf[128];
	memset(FM, 0, 128);

	FM[0] = 0xAA;	
	FM[1] = 0x55;	
	FM[2] = 0x06;	
	FM[3] = 0x00;	
	FM[4] = 0x00;	
	FM[5] = 0x00;		
	FM[6] = 0x06;
	FM[7] = 0x01;
	FM[8] = 0x01;
	FM[9] = CalBCC(FM,9);
	write(fd_rfid, FM, 10);
	//read(fd_rfid1, RBuf, 10);
	
	usleep(10000);

	return 0;
}

/*防碰撞，获取范围内最大ID*/
int PiccAnticoll(int fd_rfid)
{
	//printf("fd_rfid = %d\n", fd_rfid);

	unsigned char WBuf[128], RBuf[128],FM[128];
	//int ret_rfid=1,rec=1, i,len=0;
	fd_set rdfd;

	

	memset(WBuf, 0, 128);
	memset(RBuf,0,128);

	//buzzer(fd_rfid); //不可在这里调用蜂鸣器
	
	WBuf[0] = 0xAA;	
	WBuf[1] = 0x55;	
	WBuf[2] = 0x06;	
	WBuf[3] = 0x00;	
	WBuf[4] = 0x00;	
	WBuf[5] = 0x00;		
	WBuf[6] = 0x02;
	WBuf[7] = 0x02;
	WBuf[8] = 0x04;
	WBuf[9] = CalBCC(WBuf,9);

	// while(1)
	// {	
		tcflush (fd_rfid, TCIFLUSH);
		FD_ZERO(&rdfd);
		FD_SET(fd_rfid,&rdfd);
		
		write(fd_rfid, WBuf, 10);

		ret_rfid = select(fd_rfid + 1,&rdfd, NULL,NULL,&timeout_RFID);//NULL阻塞一直等待数据
		buzzer(fd_rfid);//蜂鸣器 只能从这开始 放在ret_rfid之前

		printf("ret_rfid = %d\n", ret_rfid);

		switch(ret_rfid)
		{

			case -1:
				perror("select error\n");
				break;
			case  0:
				len++;
				if(len == 10)
				{
					len=0;
					return -1;
				}
				perror("Timeout:");
				break;
				
			default:

				usleep(10000);
				ret_rfid = read(fd_rfid, RBuf, 128);
				if (ret_rfid < 0)
				{
					printf("ret_rfid = %d\n", ret_rfid);
					break;
				}
				if (RBuf[8] == 0x00) //应答帧状态部分为0 则获取ID 成功
				{
					printf("22222222%d\n",__LINE__);
					
					// printf("rec=%d\n",rec);
					cardid = (RBuf[9]<<24) | (RBuf[10]<<16) | (RBuf[11]<<8) | RBuf[12];
					
					return 0; 
				}
		}

	// }
	
	return -1;
}

int init_rfidSerial()
{
    struct termios myios;
    bzero(&myios,sizeof(myios));
    fd_rfid = open("/dev/USBrfid", O_RDWR | O_NOCTTY );//| O_NDELAY

    if(fd_rfid == -1)
    {
        perror("打开RFID串口失败！\n");
        return -1;
    }

    //保存当前串口的配置
    tcgetattr(fd_rfid,&myios);

    //设置串口工作在原始模式
    cfmakeraw(&myios);
    myios.c_cflag |= CLOCAL | CREAD;
	
    //设置波特率
    cfsetispeed(&myios,B9600);
    cfsetospeed(&myios,B9600);
	
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
    tcflush(fd_rfid,TCIOFLUSH);  //刷新输入输出缓冲区
	
    //串口设置使能
    tcsetattr(fd_rfid,TCSANOW,&myios);

    return 0;

}

int redRfidSerial()
{
    while(1)
	{	
        cout << __LINE__ <<endl;
		//发送A命令
		ret_rfid = PiccRequest(fd_rfid);
		if(ret_rfid == -1)  //若是请求超时退出，必须要关闭串口后，重新打开才能再次读取数据
		{
			usleep(500000);
			close(fd_rfid);
			
			//打开串口文件
			init_rfidSerial();//fd_rfid = open_serial();
			timeout_RFID.tv_sec = 1;
			timeout_RFID.tv_usec = 0;
			continue;
		}	
		
		else//(ret_rfid == 0)
		{
			printf("ok!\n");
		}
		//发送B命令 和获取卡号
		ret_rfid = PiccAnticoll(fd_rfid);

		printf("%d\n",__LINE__);
		printf("cardid:%x\n",cardid);
		
		//若获取的cardid为0，或获取id超时，则需重新发送'A'命令
		if(cardid == 0 || ret_rfid == -1) 
			continue; 
		else if(ret_rfid == 0)
		{
			printf("card ID = %x\n", cardid);  //打印cardid号
			usleep(1000000);
			//break;
		}
	}
	close(fd_rfid);
	return 0;

}

/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑RFID↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

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

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓magnetic↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
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

void redMagneticSerial_old()
{
    while(1)
    {
        usleep(50000);
        read(usbfd_magnetic, subbuf, sizeof(subbuf));
        dec2hex(subbuf[1]);
    }
}

void redMagneticSerial()
{
    while(1)
    {
        usleep(50000);
        write(usbfd_magnetic, buf_WriteMagnetic, sizeof(buf_WriteMagnetic));
        usleep(50000);
        read(usbfd_magnetic, buf_ReadMagnetic, sizeof(buf_ReadMagnetic));

        for(int i=3;i<19;i=i+2)
        {
            tmp = buf_ReadMagnetic[i];
            buf_ReadMagnetic[i] = buf_ReadMagnetic[i+1];
            buf_ReadMagnetic[i+1] = tmp;
        }
        for(int i = 3;i<19;i++)
        {
            //cout << "usbfd_magnetic[" << i << ']' << buf_ReadMagnetic[i] << endl;
            printf("buf_ReadMagnetic[%d]",i);
            printf("%x\n",buf_ReadMagnetic[i]);
            if(i == 3)
            {
                magneticMax = buf_ReadMagnetic[3];
                if(magneticMax>=0x10&&i>=3)
                {
                    magneticRank = i-2;
                }
                else
                {
                    magneticRank = 0;
                }

            }
            if(i>3)
            {            
                if(magneticMax<buf_ReadMagnetic[i])
                {               
                    magneticMax = buf_ReadMagnetic[i];
                    if(magneticMax>=0x10&&i>=4)
                    {
                        magneticRank = i-2;
                    }
                    else
                    {
                        magneticRank = 0;
                    }
                }
            }
            // MAX = buf_ReadMagnetic[3];
            // if(MAX<buf_ReadMagnetic[i])
            // {
            //     MAX = buf_ReadMagnetic[i];
            // }
        }

        printf("最大MAX:%x\n",magneticMax);
        cout << "第 " << magneticRank << "位" << endl;
    }
}

int init_magneticSerial()
{
    struct termios myios;
    bzero(&myios,sizeof(myios));
    usbfd_magnetic = open("/dev/USBmagnetic", O_RDWR | O_NOCTTY );//| O_NDELAY

    if(usbfd_magnetic == -1)
    {
        perror("打开magnetic串口失败！\n");
        return -1;
    }

    //保存当前串口的配置
    tcgetattr(usbfd_magnetic,&myios);

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
    tcflush(usbfd_magnetic,TCIOFLUSH);  //刷新输入输出缓冲区
	
    //串口设置使能
    tcsetattr(usbfd_magnetic,TCSANOW,&myios);

    return 0;
}

/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑magnetic↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓USBCAN↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/

void exit_can(int flag)
{
	//关闭设备
    VCI_CloseDevice (4, 0);

    exit(0);
}

int init_usbcanSerial()
{
    int ret;
    VCI_INIT_CONFIG init_config0;
	VCI_INIT_CONFIG init_config1;
	VCI_BOARD_INFO board_info;

	//打开设备
    fd_USBCAN = VCI_OpenDevice(4, 0, 0);
    if (fd_USBCAN == 0)
    {
        //exit_can(1);
        return 0;
    }

	bzero(&init_config0, sizeof(VCI_INIT_CONFIG));
	bzero(&init_config1, sizeof(VCI_INIT_CONFIG));
	
	//CAN0初始化参数
	//正常工作模式
	init_config0.Mode = 0;							
	//1M波特率
	init_config0.Timing0 = 0x00;							
	init_config0.Timing1 = 0x14;	

	//CAN1初始化参数
	//正常工作模式
	init_config1.Mode = 0;							
	//1M波特率
	init_config1.Timing0 = 0x00;							
	init_config1.Timing1 = 0x14;	

	
	//初始化CAN0通道
    fd_USBCAN = VCI_InitCAN(4, 0, 0, &init_config0);					
    if (fd_USBCAN == 0) {
        exit_can(1);
        return 0;
    }

	//初始化CAN1通道
    ret = VCI_InitCAN(4, 0, 1, &init_config1);
    if (ret == 0) {
        exit_can(1);
        return 0;
    }
	
	//读设备信息
	bzero(&board_info, sizeof(board_info));
    fd_USBCAN = VCI_ReadBoardInfo(4, 0, &board_info);
    if (fd_USBCAN == 0) {
        exit_can(1);
        return 0;
    }

	//启动CAN0通道
    ret = VCI_StartCAN (4, 0, 0);
    if (ret == 0) {
        exit_can(1);
        return 0;
    }

	//启动CAN1通道
    ret = VCI_StartCAN (4, 0, 1);
    if (ret == 0) 
    {
        exit_can(1);
    }
   
    printf("功能:CAN1波特率1M,接收到什么数据就回发什么数据。\n");

    return 0;
}

int cantestfun(int c)
{
    int ret;
    //VCI_CAN_OBJ recv_msg[2];

    if(c==1)
    {
        cout << "上升" << endl;
        recvlen_can = 1;

        recv_msg[0].ID = 0x00000200;
        recv_msg[0].SendType = 0;
        recv_msg[0].RemoteFlag = 0;
        recv_msg[0].ExternFlag = 0;
        recv_msg[0].DataLen = 8;
        recv_msg[0].Data[0] =0x0C; //0C 2A 0C 2A 0C 2A 0C 2A  printf("%02x \n", recv_msg[0].Data[0]);
        recv_msg[0].Data[1] =0x2A;
        recv_msg[0].Data[2] =0x0C;
        recv_msg[0].Data[3] =0x2A;
        recv_msg[0].Data[4] =0x0C;
        recv_msg[0].Data[5] =0x2A;
        recv_msg[0].Data[6] =0x0C;
        recv_msg[0].Data[7] =0x2A;

        int i,j;
		for (i=0; i<recvlen_can; i++) 
        {
			printf("ID: %08x DataLen:%02x ExternFlag:%02x RemoteFlag: %02x data:", recv_msg[i].ID, recv_msg[i].DataLen, recv_msg[i].ExternFlag, recv_msg[i].RemoteFlag);
			for (j=0; j< recv_msg[i].DataLen; j++) 
            {
				printf("%02x ", recv_msg[i].Data[j]);
			}

			printf("\n");
		}

        ret = VCI_Transmit (4, 0, 0, recv_msg, recvlen_can);
        if(ret == 0)
        {
            cout << "fail" << endl;
        }

    }

    else if (c == 0)
    {
        cout << "停止" << endl;
        recvlen_can = 1;
        recv_msg[1].ID = 0x200;
        recv_msg[0].DataLen = 8;
        recv_msg[0].ExternFlag = 0;
        recv_msg[0].RemoteFlag = 0;
        recv_msg[0].Data[0] =0xFF; //ff 43 ff 43 ff 43 ff 43
        recv_msg[0].Data[1] =0x43;
        recv_msg[0].Data[2] =0xFF;
        recv_msg[0].Data[3] =0x43;
        recv_msg[0].Data[4] =0xFF;
        recv_msg[0].Data[5] =0x43;
        recv_msg[0].Data[6] =0xFF;
        recv_msg[0].Data[7] =0x43;
        ret = VCI_Transmit (4, 0, 0, recv_msg, recvlen_can);

    }

    return 0;
}

int redUSBCanSerial()
{
    /*memset(recv_msg, 0, sizeof(VCI_CAN_OBJ)*1);*/
	while (1) 
    {
        int c;
        // recvlen_can = VCI_Receive(4, 0, 0, recv_msg, 10, 0);

        cout << "输入" << endl;
        cin >> c;

        //cantestfun(c);

	    // if(0 == recvlen_can)
        // {
        //     continue;
        // }
        // printf("---recvlen:%d\n", recvlen_can);
		// int i,j;
		// for (i=0; i<recvlen_can; i++) 
        // {
		// 	printf("ID: %08x DataLen:%02x ExternFlag:%02x RemoteFlag: %02x data:", recv_msg[i].ID, recv_msg[i].DataLen, recv_msg[i].ExternFlag, recv_msg[i].RemoteFlag);
		// 	for (j=0; j< recv_msg[i].DataLen; j++) 
        //     {
		// 		printf("%02x ", recv_msg[i].Data[j]);
		// 	}

		// 	printf("\n");
		// }

		// if(0 < recvlen_can)
		// {
		// 	ret = VCI_Transmit (4, 0, 0, recv_msg, recvlen_can);
        //     //ret = VCI_Transmit (4, 0, 1, recv_msg, recvlen_can);
		// 	printf ("-----send :%ld\n", ret);
		// }

		usleep(10000);
	}
// 	goto selfstop;

// selfstop:
//     VCI_CloseDevice (4, 0);
// 	return 0;
}

/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑USBCAN↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓USBIIC↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/

int setUSBIICMode(int fd_usbiic)
{
	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓设置USBAM0为IIC模式↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	unsigned char startiicbuf[4] ,WBuf[9];
	memset(startiicbuf, 0, 4);  //数组清空 向WBuf中写入4个0
	memset(WBuf, 0, 9);  //数组清空 向WBuf中写入9个0


	startiicbuf[0] = 0x03;	
	startiicbuf[1] = 0x00;	
	startiicbuf[2] = 0x04;	
	startiicbuf[3] = 0x00;	

	//数组发送给串口 到此设置完毕
	write(fd_usbiic, startiicbuf, 4);
	usleep(50000);
	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑设置USBAM0为IIC模式↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

	/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓读取USBAM0 iic数据的 前置数组↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
	memset(WBuf, 0, 9);  //数组清空 向WBuf中写入9个0
	WBuf[0] = 0x0C;	
	WBuf[1] = 0x00;	
	WBuf[2] = 0x09;	
	WBuf[3] = 0x00;	
	WBuf[4] = 0x00;	
	WBuf[5] = 0x48;		
	WBuf[6] = 0x00;
	WBuf[7] = 0x01;
	WBuf[8] = 0x40;

	write(fd_usbiic, WBuf,9);
	usleep(50000);

	/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑读取USBAM0 iic数据的 前置数组↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
	return 0;
}

int init_usbiicSerial()
{
	struct termios myios;
    bzero(&myios,sizeof(myios));
    fd_usbiic = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);//| O_NDELAY

    if(fd_usbiic == -1)
    {
        perror("打开usbiic串口失败！\n");
        return -1;
    }

    //保存当前串口的配置
    tcgetattr(fd_usbiic,&myios);

    //设置串口工作在原始模式
    cfmakeraw(&myios);
    myios.c_cflag |= CLOCAL | CREAD;
	
    //设置波特率
    cfsetispeed(&myios,B460800);
    cfsetospeed(&myios,B460800);
	
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
    tcflush(fd_usbiic,TCIOFLUSH);  //刷新输入输出缓冲区
	
    //串口设置使能
    tcsetattr(fd_usbiic,TCSANOW,&myios);

    //设置串口为IIC模式
    setUSBIICMode(fd_usbiic);

    return 0;
}

int redUSBIICSerial()
{
	// unsigned char WBuf[8], RBuf[128];
	int  ret, i,len=0;
	fd_set rdfd;/////f*d_set rdf*d;*******************************************

	while(1)
	{
		memset(WBuf, 0, 8);
		memset(RBuf, 0, 8);

		//读取0x40通道的数据
		WBuf[0] = 0x0D;	//0D 00 08 00 00 48 00 01
		WBuf[1] = 0x00;	
		WBuf[2] = 0x08;	
		WBuf[3] = 0x00;	
		WBuf[4] = 0x00;	
		WBuf[5] = 0x48;		
		WBuf[6] = 0x00;
		WBuf[7] = 0x01;

		write(fd_usbiic, WBuf, 8);
		usleep(50000);

		cout << __LINE__ << endl;
		ret = read(fd_usbiic, RBuf, 8);
		cout << __LINE__ << endl;
        ActualPosition = RBuf[0];
		printf("电位器数值:%f\n",ActualPosition);
        ActualPosition = ((double)180/132)*ActualPosition;
        cout << "**********************升降杆高度：" << ActualPosition << endl;
		usleep(10000);


		//读取数据
		tcflush (fd_usbiic, TCIFLUSH);
		FD_ZERO(&rdfd);
		FD_SET(fd_usbiic,&rdfd);
		
		ret = select(fd_usbiic + 1,&rdfd, NULL,NULL,&timeout_USBIIC);
		printf("ret = %d\n", ret);

        usleep(50000);

	}
}

/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑USBIIC↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/


/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓UDP↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
void init_UDP()
{
    /****************************UDP设置******************************/

    //1.UDP套接字
    //int sockfd;
    UDPsockfd = socket(AF_INET,SOCK_DGRAM,0);

    //2. 绑定IP地址，端口号到套接字上
    //struct sockaddr_in srvaddr;
    len_UDP = sizeof(srvaddr);
    bzero(&srvaddr,len_UDP);

    srvaddr.sin_family = AF_INET;
    srvaddr.sin_port = htons(/*atoi(argv[1])*/50002);
    srvaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(UDPsockfd,(struct sockaddr *)&srvaddr,len_UDP);
}

void redUDP()
{
    /****************************UDP接收自动寻路标志位AutoForwardFlag、AutoBackFlag******************************/
    cout << "这里是UDP" << endl;
    //不断接收对方给自己发送过来的信息

    while(1)
    {
        cout << "UDP收到了" << endl;
        //bzero(AutoForwardFlag,sizeof(AutoForwardFlag));
        UDPret = recvfrom(UDPsockfd,buf_receiveUDP,sizeof(buf_receiveUDP),0,(struct sockaddr *)&cliaddr,&len_UDP);
        usleep(50000);
        //AutoForwardFlag[1] = buf_receiveUDP[1];
        //AutoForwardFlag = buf_receiveUDP[1]-'0';

                             AutoFlag                   = buf_receiveUDP[0];//0x0A开启 0x0C关闭
                             AutoForwardOrBackFlag      = buf_receiveUDP[1];//0x0F前进 0x0B后退
                             DirectionFlag              = buf_receiveUDP[2];//遥控 0x0F前进 0x0B后退 0x01向左 0x02向右
        if(AutoFlag == 0x0A) ExpectPosition             = buf_receiveUDP[3];//遥控时 给的期望值 通过UpandDown
                             flag_batteryHightOrLow     = buf_receiveUDP[4];//电池电量高低

        //AutoBackFlag[1] = buf_receiveUDP[0];
        if(UDPret != -1)
        {
            //printf("%s:%s",(char *)inet_ntoa(cliaddr.sin_addr),AutoForwardFlag);
            //PolicyFlag[1] == AutoForwardFlag[1];

            // if(AutoForwardFlag[1] == '0')
            // {
            //     AutoForwardStopFlag = 1;
            // }

            // else if (AutoBackFlag[1] == '0')
            // {
            //     AutoBackStopFlag = 1;
            // }

            // if(AutoForwardFlag == 0)
            // {
            //     AutoForwardStopFlag = 1;
            // }

            // else if (AutoBackFlag == 0)
            // {
            //     //AutoBackStopFlag = 1;
            // }
        }
        usleep(50000);
    }

}
/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑UDP↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓升降杆↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
int UPandDown(double ExpectPosition)
{
    usleep(50000);
    int motorforce;
    //1.调整杆
    if(ActualPosition > ExpectPosition)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
        if(ActualPosition > ExpectPosition+30)
        {
            motorforce = -30;
        }
        else if(ActualPosition > ExpectPosition+25)
        {
            motorforce = -25;
        }
        else if(ActualPosition > ExpectPosition+20)
        {
            motorforce = -22;
        }
        else if(ActualPosition > ExpectPosition+15)
        {
            motorforce = -20;
        }

        processData_usbCanMotor(int(motorforce * 94.39 - 189.5));
    }
    else if(ActualPosition < ExpectPosition)
    {
        cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
        int motorforce = 30;
        if(ActualPosition < ExpectPosition-30)
        {
            motorforce = 40;
        }
        else if(ActualPosition < ExpectPosition-25)
        {
            motorforce = 35;
        }
        else if(ActualPosition < ExpectPosition-20)
        {
            motorforce = 28;
        }
        else if(ActualPosition < ExpectPosition-15)
        {
            motorforce = 23;
        }


        processData_usbCanMotor(int(motorforce * 94.39 - 189.5));

    }
    else if(ActualPosition >= ExpectPosition-1 && ActualPosition <= ExpectPosition+1)//ActualPosition >= ExpectPosition-5 && ActualPosition <= ExpectPosition+5)
    {
        cout << "======================================================" << endl;
        int motorforce = 0;
        processData_usbCanMotor(int(motorforce * 94.39 - 189.5));//processData_usbCanMotor
    }

    return 0;
}
/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑升降杆↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓遥控线程↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/

void ControlDirection()
{
    if(DirectionFlag == 0x0F)//向前
    {
        cout << "向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前向前" << endl;
        pub_msg.linear.x = 0.50;
        pub_msg.angular.z = 0;
        Sub2XQROS.publish(pub_msg);
        pub_msg.linear.x = 0;
        pub_msg.angular.z = 0;
    }

    else if(DirectionFlag == 0x0B)//向后
    {
        cout << "向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后向后" << endl;
        pub_msg.linear.x = -0.40;
        pub_msg.angular.z = 0;
        Sub2XQROS.publish(pub_msg);
        pub_msg.linear.x = 0;
        pub_msg.angular.z = 0;
    }

    else if (DirectionFlag == 0x01)
    {
        cout << "向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左向左" << endl;
        pub_msg.linear.x = 0;
        pub_msg.angular.z = 0.4;
        Sub2XQROS.publish(pub_msg);
        pub_msg.linear.x = 0;
        pub_msg.angular.z = 0;
    }

    else if(DirectionFlag == 0x02)
    {
        cout << "向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右向右" << endl;
        pub_msg.linear.x = 0;
        pub_msg.angular.z = -0.4;
        Sub2XQROS.publish(pub_msg);
        pub_msg.linear.x = 0;
        pub_msg.angular.z = 0;

    }
    else if(DirectionFlag == 0x0C)
    {
        pub_msg.linear.x = 0;
        pub_msg.angular.z = 0;
        Sub2XQROS.publish(pub_msg);
        pub_msg.linear.x = 0;
        pub_msg.angular.z = 0;
    }

    usleep(50000);
}

/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑遥控线程↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓Policy↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
void AutoForwardPolicy_old()
{
    if(flag_lowbattery == 0)//非低电量
    {
        if(ret_magnetic>128&&ret_magnetic<8192)
        {
            if((ret_magnetic>=64&&ret_magnetic<128) /*||(ret>=4096&&ret<8192)*/)
            {
                cout << "直行(右)" << endl;
                pub_msg.linear.x = 0.15;
                pub_msg.angular.z = -0.10;
                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }
            else if(/*(ret>=64&&ret<128) ||*/(ret_magnetic>=4096&&ret_magnetic<8192))
            {
                cout << "直行(左)" << endl;
                pub_msg.linear.x = 0.15;
                pub_msg.angular.z = 0.10;
                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }


            else if((ret_magnetic>=128&&ret_magnetic<256) /*|| (ret>=1024&&ret<4096)*/)
            {
                cout << "直行(左)" << endl;
                pub_msg.linear.x = 0.19;
                pub_msg.angular.z = 0.06;
                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;

                //turn_flag = 0;
            }

            else if(/*(ret>=128&&ret<256) ||*/ (ret_magnetic>=1024&&ret_magnetic<4096))
            {
                cout << "直行(右)" << endl;
                pub_msg.linear.x = 0.19;
                pub_msg.angular.z = -0.06;
                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;

                //turn_flag = 0;
            }

            else if((ret_magnetic>=256&&ret_magnetic<1792) /*|| (ret>=512&&ret<1024)*/)//(ret>=64&&ret<127) || (ret>=8192&&ret<16384)
            {
                cout << "直行(右)" << endl;
                pub_msg.linear.x = 0.23;
                pub_msg.angular.z = 0.01;
                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;

                //turn_flag = 0;
            }
            else if(/*(ret>=256&&ret<512) ||*/ (ret_magnetic>=1792&&ret_magnetic<16384))//(ret>=64&&ret<127) || (ret>=8192&&ret<16384)
            {
                cout << "直行" << endl;
                pub_msg.linear.x = 0.18;
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

        else if(ret_magnetic>=16384)//et>=32768
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

        else if(ret_magnetic<=16&&ret_magnetic!=0)
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
        else if(ret_magnetic == 0 )
        {
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
            Sub2XQROS.publish(pub_msg);

            cout <<"*****************停了*******"<< endl;

            //此时要 1.判断rfid的数值
            //      2.读取电位器的数值 16进制转换10进制 
            //      3.数值通过can控制电机、同时也通过udp发送给手机 实时监控上升杆的位置  
            //      4.
            char buf[10] = "4187e1a";
            snprintf(cardidstr,sizeof(cardidstr),"%x",cardid);//snprintf 把字符串数字转成可计算的数字

            cout << "cardid = " << cardid << endl;
            cout << "cardidstr:" << cardidstr << endl;

            if (flag_lowbattery == 0)//非低电量时 
            {
                if(cardid == 0x41e801A)
                {
                    ExpectPosition = 150;
                    cout << "这里是4187e1a-------------------------------------------------------" << endl;

                    UPandDown(ExpectPosition);
                }

                else if(cardid == 0x4bd5f1a)
                {
                    ExpectPosition = 120;
                    UPandDown(ExpectPosition);
                }
            }
        }
    }

    else if(flag_lowbattery == 1)//低电量时 
    {
        if (cardid == 81715738);
    }
}

void AutoBackPolicy_old()
{
    if(ret_magnetic>128&&ret_magnetic<8192)
    {
        if((ret_magnetic>=64&&ret_magnetic<128) /*||(ret>=4096&&ret<8192)*/)
        {
            cout << "后退检测到(右)" << endl;
            pub_msg.linear.x = -0.15;
            pub_msg.angular.z = 0.10;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }
        else if(/*(ret>=64&&ret<128) ||*/(ret_magnetic>=4096&&ret_magnetic<8192))
        {
            cout << "后退检测到(左)" << endl;
            pub_msg.linear.x = -0.15;
            pub_msg.angular.z = -0.10;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }


        else if((ret_magnetic>=128&&ret_magnetic<256) /*|| (ret>=1024&&ret<4096)*/)
        {
            cout << "直后退检测到(左)" << endl;
            pub_msg.linear.x = -0.19;
            pub_msg.angular.z = -0.06;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;

            //turn_flag = 0;
        }

        else if(/*(ret>=128&&ret<256) ||*/ (ret_magnetic>=1024&&ret_magnetic<4096))
        {
            cout << "后退检测到(右)" << endl;
            pub_msg.linear.x = -0.19;
            pub_msg.angular.z = 0.06;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;

            //turn_flag = 0;
        }

        else if((ret_magnetic>=256&&ret_magnetic<512) /*|| (ret>=512&&ret<1024)*/)//(ret>=64&&ret<127) || (ret>=8192&&ret<16384)
        {
            cout << "后退检测到(右)" << endl;
            pub_msg.linear.x = -0.23;
            pub_msg.angular.z = -0.02;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;

            //turn_flag = 0;
        }
        else if(/*(ret>=256&&ret<512) ||*/ (ret_magnetic>=512&&ret_magnetic<1024))//(ret>=64&&ret<127) || (ret>=8192&&ret<16384)
        {
            cout << "后退" << endl;
            pub_msg.linear.x = -0.23;
            pub_msg.angular.z = 0.02;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;

            //turn_flag = 0;
        }
    }

    else if(ret_magnetic>=8192)//et>=32768
    {
        cout << "回退检测到最左" << endl;
        pub_msg.linear.x = -0.08;
        pub_msg.angular.z = -0.35;
        Sub2XQROS.publish(pub_msg);
        pub_msg.angular.z = 0;
        pub_msg.linear.x = 0;

        turn_flag = 1;
    }

    else if(ret_magnetic<=128&&ret_magnetic!=0)
    {
        cout << "后退检测到最右" << endl;
        cout << __LINE__ << endl;
        pub_msg.linear.x = -0.08;
        pub_msg.angular.z = 0.35;
        Sub2XQROS.publish(pub_msg);
        pub_msg.angular.z = 0;
        pub_msg.linear.x = 0;

        turn_flag = 1;
    }
    else if(ret_magnetic == 0)
    {
        pub_msg.linear.x = 0;
        pub_msg.angular.z = 0;
        Sub2XQROS.publish(pub_msg);
    }
}

void AutoForwardPolicy()
{
    if(flag_batteryHightOrLow == 0x48)//高电量
    {
        if(magneticRank == 7 || magneticRank == 8 || magneticRank == 9 || magneticRank == 10)
        {
            cout << "直行" << endl;
            pub_msg.linear.x = 0.3;
            pub_msg.angular.z = 0.0;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }
        else if(magneticRank == 5 || magneticRank == 6 || magneticRank == 11 || magneticRank == 12)
        {
            if(magneticRank == 5 || magneticRank == 6)
            {
                cout << "向右" << endl;
                pub_msg.linear.x = 0.3;
                pub_msg.angular.z = -0.08;
            }
            else if(magneticRank == 11 || magneticRank == 12)
            {
                cout << "向左" << endl;
                pub_msg.linear.x = 0.15;
                pub_msg.angular.z = 0.08;
            }

            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }
        else if(magneticRank == 3 || magneticRank == 4 || magneticRank == 13 || magneticRank == 14)
        {
            if(magneticRank == 3 || magneticRank == 4)
            {
                cout << "向右" << endl;
                pub_msg.linear.x = 0.15;
                pub_msg.angular.z = -0.15;
            }
            else if(magneticRank == 13 || magneticRank == 14)
            {
                cout << "向左" << endl;
                pub_msg.linear.x = 0.15;
                pub_msg.angular.z = 0.15;
            }

            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }
        else if(magneticRank == 1 || magneticRank == 2 || magneticRank == 15 || magneticRank == 16)
        {
            if(magneticRank == 1 || magneticRank == 2)
            {
                cout << "向右" << endl;
                pub_msg.linear.x = 0.10;
                pub_msg.angular.z = -0.4;
            }
            else if(magneticRank == 15 || magneticRank == 16)
            {
                cout << "向左" << endl;
                pub_msg.linear.x = 0.10;
                pub_msg.angular.z = 0.4;
            }

            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }

        else if(magneticRank == 0)
        {
            cout << "暂停" << endl;
            pub_msg.linear.x = 0.0;
            pub_msg.angular.z = 0.0;
        }

        base = 0;
    }
/*    else if (flag_batteryHightOrLow == 0x4C)//低电量时
    {
        if(magneticRank == 7 || magneticRank == 8 || magneticRank == 9 || magneticRank == 10)
        {
            cout << "直行" << endl;
            pub_msg.linear.x = 0.3;
            pub_msg.angular.z = 0.0;
            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }
        else if(magneticRank == 5 || magneticRank == 6)
        {
            if(magneticRank == 5 || magneticRank == 6)
            {
                cout << "向右" << endl;
                pub_msg.linear.x = 0.3;
                pub_msg.angular.z = -0.08;
            }

            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }
        else if(magneticRank == 3 || magneticRank == 4)
        {
            if(magneticRank == 3 || magneticRank == 4)
            {
                cout << "向右" << endl;
                pub_msg.linear.x = 0.15;
                pub_msg.angular.z = -0.15;
            }

            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }
        else if(magneticRank == 1 || magneticRank == 2)
        {
            if(magneticRank == 1 || magneticRank == 2)
            {
                cout << "向右" << endl;
                pub_msg.linear.x = 0.10;
                pub_msg.angular.z = -0.4;
            }

            Sub2XQROS.publish(pub_msg);
            pub_msg.linear.x = 0;
            pub_msg.angular.z = 0;
        }

        else if(magneticRank == 0)
        {
            cout << "暂停" << endl;
            pub_msg.linear.x = 0.0;
            pub_msg.angular.z = 0.0;
        }
    }*/
    else if (flag_batteryHightOrLow == 0x4C)//低电量时
    {
        if(base == 0)
        {
            if(magneticRank == 4 || magneticRank == 5 || magneticRank == 6 || magneticRank == 7 || magneticRank == 8 
            || magneticRank == 9 || magneticRank == 10 || magneticRank == 11 || magneticRank == 12 )
            {
                cout << "向右" << endl;
                pub_msg.linear.x = 0.25;
                pub_msg.angular.z = -0.09;
                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }
            else if (magneticRank == 16)
            {
                cout << "向左" << endl;
                pub_msg.linear.x = 0.20;
                pub_msg.angular.z = 0.08;
                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }
            else if (magneticRank == 13 || magneticRank == 14 || magneticRank == 15)
            {
                cout << "直行" << endl;
                pub_msg.linear.x = 0.3;
                pub_msg.angular.z = 0.0;
                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;

            }
            else if (magneticRank == 1 || magneticRank == 2 || magneticRank == 3)
            {
                cout << "向右" << endl;
                pub_msg.linear.x = 0.10;
                pub_msg.angular.z = -0.4;
                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
                base = 1;
            }
        }
        else if (base == 1)
        {
            if(magneticRank == 7 || magneticRank == 8 || magneticRank == 9 || magneticRank == 10)
            {
                cout << "直行" << endl;
                pub_msg.linear.x = 0.3;
                pub_msg.angular.z = 0.0;
                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }
            else if(magneticRank == 5 || magneticRank == 6 || magneticRank == 11 || magneticRank == 12)
            {
                if(magneticRank == 5 || magneticRank == 6)
                {
                    cout << "向右" << endl;
                    pub_msg.linear.x = 0.3;
                    pub_msg.angular.z = -0.08;
                }
                else if(magneticRank == 11 || magneticRank == 12)
                {
                    cout << "向左" << endl;
                    pub_msg.linear.x = 0.15;
                    pub_msg.angular.z = 0.08;
                }

                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }
            else if(magneticRank == 3 || magneticRank == 4 || magneticRank == 13 || magneticRank == 14)
            {
                if(magneticRank == 3 || magneticRank == 4)
                {
                    cout << "向右" << endl;
                    pub_msg.linear.x = 0.15;
                    pub_msg.angular.z = -0.15;
                }
                else if(magneticRank == 13 || magneticRank == 14)
                {
                    cout << "向左" << endl;
                    pub_msg.linear.x = 0.15;
                    pub_msg.angular.z = 0.15;
                }

                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }
            else if(magneticRank == 1 || magneticRank == 2 || magneticRank == 15 || magneticRank == 16)
            {
                if(magneticRank == 1 || magneticRank == 2)
                {
                    cout << "向右" << endl;
                    pub_msg.linear.x = 0.10;
                    pub_msg.angular.z = -0.4;
                }
                else if(magneticRank == 15 || magneticRank == 16)
                {
                    cout << "向左" << endl;
                    pub_msg.linear.x = 0.10;
                    pub_msg.angular.z = 0.4;
                }

                Sub2XQROS.publish(pub_msg);
                pub_msg.linear.x = 0;
                pub_msg.angular.z = 0;
            }

            else if(magneticRank == 0)
            {
                cout << "暂停" << endl;
                pub_msg.linear.x = 0.0;
                pub_msg.angular.z = 0.0;
            }
        }
    }
}

/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑Policy↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

int main(int argc,char *argv[])
{
    uint16_t crc = 0;

    //*********************************************************磁传感器magnetic串口初始化*********************************/
	init_magneticSerial();

    /**********************************************************RFID串口初始化*******************************************/
    init_rfidSerial();

    /**********************************************************USBCAN串口初始化*****************************************/
    init_usbcanSerial();

    /**********************************************************USBIIC串口初始化*****************************************/
    init_usbiicSerial();//初始化串口

    /*************************************************************初始化UDP********************************************/
    init_UDP();

    //*******************************************************ROS设置初始化********************************************************
    // ROS节点初始化
    ros::init(argc, argv, "test");
    // 创建节点句柄
    ros::NodeHandle n;

    ros::Rate loop_rate(20);
    Sub2XQROS = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

/*****************************************************************************功能函数**********************************************/
    //thread Magneticthread(redMagneticSerial_old);//读取磁力传感器线程
    //thread processDATAthread(processData_magnetic_old);//处理磁力传感器数据
    // thread USBCANthread(redUSBCanSerial);//

    thread Magneticthread(redMagneticSerial);//读取磁力传感器线程
    // thread Rfidthread(redRfidSerial);//读取RFID数据线程
    thread USBIICthread(redUSBIICSerial);//读取USBIIC串口信息
    thread UDPthread(redUDP);//读取UDP信息

    thread ControlDirectionthread(ControlDirection);//遥控线程
    
    while(n.ok())
    {

        //自动前进循迹
        if(AutoFlag == 0x0A) //if(AutoForwardFlag[1] == '1' /*&& AutoBackFlag[1] == '0'*/ )
        {
            ret_magnetic = strtol(hex2decbuf,&ptr,0);//开启自动循迹 //hex2decbuf 磁传感器发过来的值 &ptr字符 自动转换0x 为十进制
            cout << __LINE__ << endl;
            if (AutoForwardOrBackFlag == 0x0F)
            {
                //AutoForwardPolicy_old();
                AutoForwardPolicy();
            }
            else if (AutoForwardOrBackFlag == 0x0B)
            {
                //AutoBackPolicy_old();
            }
        }

        //停止自动寻迹 
        else if(AutoFlag == 0x0C)
        {
            ret_magnetic = 0;//停止自动寻迹后 磁力传感器接收的数据清零  ******这里是重点*****
            ControlDirection();//遥控
        }

        // cout << "ret_magnetic=" << ret_magnetic << endl;


        loop_rate.sleep();
        ros::spinOnce();

    }

    close(usbfd_magnetic);
    close(fd_rfid);
    close(fd_USBCAN);
    close(fd_usbiic);
    close(UDPsockfd);
    VCI_CloseDevice (4, 0);

    Magneticthread.detach();
    // Rfidthread.detach();
                        // USBCANthread.detach();
    // USBIICthread.detach();
    UDPthread.detach();
    //processDATAthread.detach();

    return 0;

}
