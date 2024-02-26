#ifndef _MyUart_h
#define _MyUart_h
#include "stop_watch.hpp"
#include "uart_test.hpp"
#include "headfile.hpp"
#include <iostream> 
#include <unistd.h>
using namespace std;
/*
函数简介：串口通信-发送信号的初始化
参数说明：void
返回结果：初始化完毕：0    初始化失败：非0
备注信息：
*/

//extern uint8 send[14];//向下位机发送的数组
// extern int controldata;//向下位机发送的打角行数据
// extern int motordata;//向下位机发送电机控制数据
extern int fd;
extern std::shared_ptr<Driver> driver;

int usb_uart_send_init(void);
/*
函数简介：串口通信-接收的初始化
参数说明：void
返回结果：初始化完毕：0    初始化失败：非0
备注信息：
*/
int usb_uart_recv_init(void);
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
int usb_uart_5byte_send(uint8 *flag_to_lowercomputer, int duty_Motor, int duty_SMotor );
/*
函数简介：串口通信封装函数，函数内部完成对用户输入数据的编码和发送，用户只需输入要发送的数据即可
参数说明：*flag_to_lowercomputer:长度为14的标志数组，duty_Motor:电机的目标期望(-126~127)，duty_SMotor:舵机的目标期望(-126~127)
返回结果：发送成功：0    发送失败：1
备注信息：2.0版本
*/
int usb_uart_5byte_send2_0(uint8 *flag_to_lowercomputer, int duty_Motor, int duty_SMotor );
/*
函数简介：串口通信封装函数，函数内部完成对用户输入数据的编码和发送，用户只需输入要发送的数据即可
参数说明：*flag_to_lowercomputer:长度为14的标志数组，duty_Motor:电机的目标期望(-126~127)，duty_SMotor:舵机的目标期望(-126~127)
返回结果：发送成功：1    发送失败：0
备注信息：3.0版本
*/
int usb_uart_5byte_send_3_0(int fd, uint8 *flag_to_lowercomputer, int duty_Motor, int duty_SMotor);

int usb_uart_5byte_send_3_1(int fd, uint8 *flag_to_lowercomputer, int duty_Motor, int duty_SMotor);

int usb_uart_send_4_0(int fd, uint8 *flag_to_lowercomputer, int duty_Motor, int duty_SMotor, int distance, float slope);

uint8 uart_hihgercomputer_synergy(void);

int set_interface_attribs(int fd, int speed);
/*
函数简介：linux系统下对于串口发送的时序控制
参数说明：fd端口地址(端口名)，控制信号
返回结果：void
备注信息：当控制信号置1时，
*/
void set_mincount(int fd, int mcount);

#endif