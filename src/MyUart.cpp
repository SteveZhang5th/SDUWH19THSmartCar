
#include "stop_watch.hpp"
#include "uart_test.hpp"
#include "headfile.hpp"
//#include "MyUart.hpp"

/*
函数简介：串口通信-发送信号的初始化
参数说明：void
返回结果：初始化完毕：0    初始化失败：非0
备注信息：
*/

//uint8 send[14] = {0};//向下位机发送的数组
// int controldata = 0;//向下位机发送的打角行数据
// int motordata = 28;//向下位机发送电机控制数据
int fd;
std::shared_ptr<Driver> driver = nullptr;

int usb_uart_send_init(void){
  driver = std::make_shared<Driver>("/dev/ttyUSB0", BaudRate::BAUD_460800);
  if (driver == nullptr) {
  std::cout << "Create Driver Error ." << std::endl;
  return -1;
  }
  //串口初始化，打开串口设备及配置串口数据格式
  int ret = driver->open();
  
  if (ret != 0){
  std::cout << "Driver Open failed ." << std::endl;
  return -1;
  }
  cout<<"init send successful"<<endl;
  return 0;
}

/*
函数简介：串口通信-接收的初始化
参数说明：void
返回结果：初始化完毕：0    初始化失败：非0
备注信息：
*/
int usb_uart_recv_init(void){
    int ret = 0;
    //与下文接收一个字节的数据并打印相关
    //size_t timeout_ms = 5000;//阻塞时间5000ms。
    //uint8_t recv_data;

    //USB转串口的设备名为/dev/ttyUSB0
    driver = std::make_shared<Driver>("/dev/ttyUSB0", BaudRate::BAUD_460800);
    if (driver == nullptr) {
    std::cout << "Create Driver Error ." << std::endl;
    return -1;
    }
    //串口初始化，打开串口设备及配置串口数据格式
    ret = driver->open();
    cout<<"ret = "<<ret<<endl;
    if (ret != 0){
    std::cout << "Driver Open failed ." << std::endl;
    return -1;
    }
    cout<<"init recive successful"<<endl;
    return 0;
}
/*
函数简介：串口通信封装函数，函数内部完成对用户输入数据的编码和发送，用户只需输入要发送的数据即可
参数说明：*flag_to_lowercomputer:长度为14的标志数组，duty_Motor:电机的目标期望(-126~127)，duty_SMotor:舵机的目标期望(-126~127)
返回结果：发送成功：0    发送失败：1
备注信息：
通信格式：(用户使用该函数只需根据上文函数简介、参数说明、返回结果即可,以下内容是介绍通信的编码格式)
每一帧信号共有5byte，分别为：
起始位8位：0xFF(八位全是1)，用于下位机判断每一帧的开始，下位机将根据该信号向上位机发送应答信号
标志菜单0-8位：高七位为空闲标志位存储空间，最低位必须为0
标志菜单1-8位：高七位为空闲标志位存储空间，最低位必须为0
电机速度8位：最高位表正负,0为正1为负;低七位二进制表示数值大小;!!!特别注意:不可以将电机速度期望设置为-127，因-127编码结果为0xFF,与起始位冲突
多机方位8位：最高位表正负,0为正1为负;低七位二进制表示数值大小;!!!特别注意:不可以将舵机方位期望设置为-127，因-127编码结果为0xFF,与起始位冲突
*/
int usb_uart_5byte_send(uint8 *flag_to_lowercomputer, int duty_Motor, int duty_SMotor ){
  uint8 i;//内部计数变量，当前默认flag菜单的长度为14
  uint8 message[5] = {0xFF};
  //uint8 answer_of_lowercomputer = 0;
  uint8 recv_message = 1;
  
  for(i = 0; i<7; i++){//将前七个标志位转化为一个uint8类型的变量，flag_to_lowercomputer[0]为最高位，uint8的最低位为0
    message[1] = 2*message[1] + flag_to_lowercomputer[i];
  }
  for(i = 7; i<13; i++){//将后七个标志位转化为一个uint8类型的变量，flag_to_lowercomputer[7]为最高位，uint8的最低位为0
    message[2] = 2*message[2] + flag_to_lowercomputer[i];
  }

  if(duty_Motor>=0 && duty_Motor<=127 ){//电机目标在0~127之间时，message[3]的最高位为0
    message[3] = duty_Motor;
  }else if(duty_Motor<0 && duty_Motor>= -126){//电机目标在-126~0之间时，message[3]的最高位为1
    message[3] = duty_Motor;
    //cout<<message[3]<<endl;
  }else 
  message[3] = 0;//如果出现错位，则置零

  if(duty_SMotor>=0 && duty_SMotor<=127 ){//舵机目标在0~127之间时，message[4]的最高位为0
    message[4] = duty_SMotor;
  }else if(duty_SMotor<0 && duty_SMotor>= -126){//舵机目标在-126~0之间时，message[4]的最高位为1
    message[4] = (uint8)(duty_SMotor);
  }else {
  message[4] = 0;//如果出现错误，则置零
  
  }

  while(driver->recvdata(recv_message,0) != 0);

  if(recv_message == 255){
    i = 1;
    while(i<5){
      usleep(100);//500微秒
      driver->senddata(message[i]);//发送数据
      i++;
    }

    recv_message = 0;
    return 0;//如果成功发送了5byte数据，则返回0

  }
  return 1;//如果接受的下位机应答不是0xFF，或是因为异常情况未能完整发送5byte数据，则返回1

}


/*
函数简介：串口通信封装函数，函数内部完成对用户输入数据的编码和发送，用户只需输入要发送的数据即可
参数说明：*flag_to_lowercomputer:长度为14的标志数组，duty_Motor:电机的目标期望(-126~127)，duty_SMotor:舵机的目标期望(-126~127)
返回结果：发送成功：0    发送失败：1
备注信息：2.0版本
*/
int usb_uart_5byte_send2_0(uint8 *flag_to_lowercomputer, int duty_Motor, int duty_SMotor ){
  uint8 i;//内部计数变量，当前默认flag菜单的长度为14
  uint8 message[5] = {0xFF};
  //uint8 answer_of_lowercomputer = 0;
  //uint8 recv_message = 1;
  
  for(i = 0; i<7; i++){//将前七个标志位转化为一个uint8类型的变量，flag_to_lowercomputer[0]为最高位，uint8的最低位为0
    message[1] = 2*message[1] + flag_to_lowercomputer[i];
  }
  for(i = 7; i<13; i++){//将后七个标志位转化为一个uint8类型的变量，flag_to_lowercomputer[7]为最高位，uint8的最低位为0
    message[2] = 2*message[2] + flag_to_lowercomputer[i];
  }

  if(duty_Motor>=0 && duty_Motor<=127 ){//电机目标在0~127之间时，message[3]的最高位为0
    message[3] = duty_Motor;
  }else if(duty_Motor<0 && duty_Motor>= -126){//电机目标在-126~0之间时，message[3]的最高位为1
    message[3] = duty_Motor;
    //cout<<message[3]<<endl;
  }else 
  message[3] = 0;//如果出现错位，则置零

  if(duty_SMotor > 127)
    duty_SMotor = 127;
  else if(duty_SMotor < -126)
    duty_SMotor = -126;

  if(duty_SMotor>=0 && duty_SMotor<=127 ){//舵机目标在0~127之间时，message[4]的最高位为0
    message[4] = duty_SMotor;
  }else if(duty_SMotor<0 && duty_SMotor>= -126){//舵机目标在-126~0之间时，message[4]的最高位为1
    message[4] = (uint8)(duty_SMotor);
  }else {
  message[4] = 0;//如果出现错误，则置零
  
  }
  
driver->senddata(message[0]);//发送判别位
//usleep(10);//500微秒
driver->senddata(message[1]);//发送判别位
// usleep(10);//500微秒
driver->senddata(message[2]);//发送判别位
// usleep(10);//500微秒
driver->senddata(message[3]);//发送判别位
// usleep(10);//500微秒
driver->senddata(message[4]);//发送判别位
return 1;//如果接受的下位机应答不是0xFF，或是因为异常情况未能完整发送5byte数据，则返回1

}

/*
函数简介：串口通信封装函数，函数内部完成对用户输入数据的编码和发送，用户只需输入要发送的数据即可
参数说明：*flag_to_lowercomputer:长度为14的标志数组，duty_Motor:电机的目标期望(-126~127)，duty_SMotor:舵机的目标期望(-126~127)
返回结果：发送成功：1    发送失败：0
备注信息：3.0版本
*/
int usb_uart_5byte_send_3_0(int fd, uint8 *flag_to_lowercomputer, int duty_Motor, int duty_SMotor)
{
  uint8 i;//内部计数变量，当前默认flag菜单的长度为14
  uint8 message[5] = {0xFF};
  //uint8 answer_of_lowercomputer = 0;
  //uint8 recv_message = 1;

  
  
  for(i = 0; i<7; i++){//将前七个标志位转化为一个uint8类型的变量，flag_to_lowercomputer[0]为最高位，uint8的最低位为0
    message[1] = 2*message[1] + flag_to_lowercomputer[i];
  }
  for(i = 7; i<13; i++){//将后七个标志位转化为一个uint8类型的变量，flag_to_lowercomputer[7]为最高位，uint8的最低位为0
    message[2] = 2*message[2] + flag_to_lowercomputer[i];
  }

  if(duty_SMotor > 120)
    duty_SMotor = 120;
  else if(duty_SMotor < -120)
    duty_SMotor = -120;

  if(duty_Motor>=0 && duty_Motor<=127 ){//电机目标在0~127之间时，message[3]的最高位为0
    message[3] = duty_Motor;
  }else if(duty_Motor<0 && duty_Motor>= -126){//电机目标在-126~0之间时，message[3]的最高位为1
    message[3] = duty_Motor;
    //cout<<message[3]<<endl;
  }else 
  message[3] = 0;//如果出现错位，则置零

  if(duty_SMotor>=0 && duty_SMotor<=127 ){//舵机目标在0~127之间时，message[4]的最高位为0
    message[4] = duty_SMotor;
  }else if(duty_SMotor<0 && duty_SMotor>= -126){//舵机目标在-126~0之间时，message[4]的最高位为1
    message[4] = (uint8)(duty_SMotor);
  }else {
  message[4] = 0;//如果出现错误，则置零
  }

  i = write(fd, message, 5);
  //i = write(fd, message, 5);
  memset(flag_to_lowercomputer,0,14);
  
  

  return i==5;
}


uint8 uart_hihgercomputer_synergy(void){
  uint32 i = 0;//计数单位
  uint8 answer_from_lowercomputer = 0xFE;//用于存放下位机准备完毕应答的变量

  while(answer_from_lowercomputer != 1)//当上位机查询下位机应答为“未准备好”时，将进入以下循环，并且等待超时后程序也会再次回到这里判断
  {                                    //是否接收到有效应答，如果没有将重复发送数据,如果应答是有效的，将彻底跳出这个两层嵌套循环
    driver->senddata(0x01);//发送上位机初始化完毕的指示
    while(driver->recvdata(answer_from_lowercomputer, 0) != 0)//查询下位机是否有回复，如果没有，则进入等待循环
    {
      i++;//每查询一次，计数单位加一
      if(i > 480000)//如果等待时间过长，则判断为下位机没有收到上位机的指示
      {
        i = 0;//将计数单位复位
        break;//退出该while循环，并进入到外层while循环
      }
    }
  }

  return 1;//由于只有应答是有效的，程序才会跳出上文的两层循环，来到这里，故此时函数返回同步成功
}

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    //cflag的设置最终结果为8位数据位，无数据屏蔽，1位停止位，启动接收，无奇偶校验
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0.01;
    //   û       С
    // struct serial_struct serial; 
    // serial.xmit_fifo_size = 1024*1024;
    // ioctl(fd, TIOCSSERIAL, &serial); 

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}


/*
函数说明：USB转UART串口通信，第3.1版本
参数列表：USB通道地址fd，上位机需要发送的标志指针，电机期望速度，舵机期望打角
返回结果：当发送成功时返回1，发送失败时返回0
备注信息：对于3.1版本和3.0版本之间的区别，则主要存在于对于标志位的处理方式，以往3.0版本中的标志位，对于上位机用户而言，标志位
在14个8位uint型数组中从0到13依次排列，前7个为一组，后7个为一组，每一组中，例如：0-6编号中，将依次转换为1bit排列于message[3]中，
其中，message[3]的第7位始终为0，第0位到第6位依次为编号6到0标志位.但是现在为了便于理解以及顺应逻辑习惯，第0位到第6位依次为编号0到6号标志位。
*/
int usb_uart_5byte_send_3_1(int fd, uint8 *flag_to_lowercomputer, int duty_Motor, int duty_SMotor)
{
    uint8 i;//内部计数变量，当前默认flag菜单的长度为14
    uint8 message[5] = {0xFF};

    for(i = 6; i >= 0; i--)
    {
        message[1] += flag_to_lowercomputer[i]<<i;
    }
    for(i = 13; i >= 7; i--)
    {
        message[2] += flag_to_lowercomputer[i]<<(i-7);
    }

    if(duty_SMotor > 120)
        duty_SMotor = 120;
    else if(duty_SMotor < -120)
        duty_SMotor = -120;

    if(duty_Motor>=0 && duty_Motor<=127 ){//电机目标在0~127之间时，message[3]的最高位为0
        message[3] = duty_Motor;
    }else if(duty_Motor<0 && duty_Motor>= -126){//电机目标在-126~0之间时，message[3]的最高位为1
        message[3] = duty_Motor;
    //cout<<message[3]<<endl;
    }else 
    message[3] = 0;//如果出现错位，则置零

    if(duty_SMotor>=0 && duty_SMotor<=127 ){//舵机目标在0~127之间时，message[4]的最高位为0
        message[4] = duty_SMotor;
    }else if(duty_SMotor<0 && duty_SMotor>= -126){//舵机目标在-126~0之间时，message[4]的最高位为1
        message[4] = (uint8)(duty_SMotor);
    }else {
        message[4] = 0;//如果出现错误，则置零
    }

    i = write(fd, message, 5);
    //i = write(fd, message, 5);
    memset(flag_to_lowercomputer,0,14);
  
  

  return i==5;
}


/*
函数说明：USB转UART串口通信，版本4.0
参数列表：USB端口地址fd,上位机需要发送的标志位，电机期望速度，舵机期望打角，车头前赛道最远距离，赛道拟合中线的斜率
返回结果：如果发送成功则返回1，发送失败返回0
备注信息：4.0版本中，通信帧进行了扩展，电机的数据长度从8位扩展到16位，舵机保持不变为8位，引入了新的通信内容最远距离distance(8位)，
以及赛道拟合中线的斜率slope(16位)
*/
int usb_uart_send_4_0(int fd, uint8 *flag_to_lowercomputer, int duty_Motor, int duty_SMotor, int distance, float slope)
{
    uint8 i;
    uint8 Message[9] = {0xFF};

    Message[0] = 0xFF;
    for(i = 6; i >= 0; i--)
    {
        Message[1] += flag_to_lowercomputer[i]<<i;
    }
    for(i = 13; i >= 7; i--)
    {
        Message[2] += flag_to_lowercomputer[i]<<(i-7);
    }

    if(duty_Motor > 0)
    {
        Message[3] = duty_Motor/100;
        Message[4] = duty_Motor%100;
    }
    else
    {
        Message[3] = -duty_Motor/100;
        Message[4] = duty_Motor%100;
    }

    if(duty_SMotor > 120)
        duty_SMotor = 120;
    else if(duty_SMotor < -120)
        duty_SMotor = -120;
    Message[5] = duty_SMotor;

    if(distance >= 254)
        distance = 254;
    Message[6] = distance;

    slope *= 10000;
    if(slope > 0)
    {
        Message[7] = (int)slope/100;
        Message[8] = (int)slope%100;
        Message[7] &= 0x7F;
    }
    else
    {
        slope = -slope;
        Message[7] = (int)slope/100;
        Message[8] = (int)slope%100;
        Message[7] |= 0x80;
    }
    
    for(i = 0; i < 9; i++)
    {
        Message[i] = 0xFF;
    }
    i = write(fd, Message, 9);
    //i = write(fd, message, 5);
    memset(flag_to_lowercomputer,0,14);

    return i==9;
}

/*
函数简介：linux系统下对于串口发送的时序控制
参数说明：fd端口地址(端口名)，控制信号
返回结果：void
备注信息：当控制信号置1时，
*/
void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 0.1;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}
