该备份使用ARM开发板和小强中控形成局域网 开发板192.168.3.242 小强192.168.3.211

//src_OnlyMagnetic 文件夹下
magnetic.cpp 是磁传感器的代码 设备名字是 /dev/USBmagnetic
    磁传感器串口线程//接受串口信息 控制小车按磁轨运行
    UDP线程//接收上位机的自动行驶标志位


//src_OnlyROS&UDP 文件夹下
ROS&UDP.cpp 是上位机通过UDP控制can电机 上下 端口50001
    UDPsend发送线程 //使用键盘控制时 小强通过ROS 的XQROS2UDPcallBack话题 给这个程序 然后通过UDP发送给开发板 


//src_OnlyROS&UDP&QTUDP 文件夹下
src_OnlyROS&UDP&QTUDP.cpp 
    UDPsend发送线程 //使用键盘控制时 小强通过ROS 的XQROS2UDPcallBack话题 给这个程序 储存在XQROS2UDPbuf[8]中（XQROS2UDPbuf[0] = sub_msg.data）  然后通过UDPsend发送给开发板 sendto（）
    UDPreceive接收线程 //接收上位机 上下左右升降指令 上下左右再通过ROS发送给开发板控制can电机

