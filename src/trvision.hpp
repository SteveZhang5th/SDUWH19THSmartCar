#ifndef _trvision_h
#define _trvision_h
/*****************************************************************************
Copyright: 山东大学（威海）智能车实验室 山魂八队（内部资源）
File name: trvision.hpp（traditional vision)
Description: 该文件中主要包含八临域巡线函数，完成巡边任务，获得道路边界的结构体和一维数组
Author: 张宇
Version: 1.0
Date: 2023.8.17
History: 着重加深了封顶行的应用，对于道路边界不再有很死的要求不越过中线，对于转弯时的获取的信息不再有很多丢失
*****************************************************************************/

#include "headfile.hpp"
#include "MyMath.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

using namespace cv;

#define CamH 90   //图像高度
#define CamW 150   //图像宽度
#define ReCamH 80  //逆透视变换后图像高度
#define ReCamW 110 //逆透视变换后图像宽度
#define Mid CamW/2 //宏定义标定的图像中线
#define WHITE 254   //定义二值化白色像素点的值
#define BLACK 1 //定义二值化黑色像素点的值
#define     func_limit(x, y)        ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))
#define pi 3.1415926f

extern int Threshold;//存储每幅灰度图像计算出的二值化阈值
extern uint8 toplinearry[14];//寻找有效封顶行所使用的数组，初始化为图像最顶端
extern uint8 Topline;//封顶行
extern uint8 images[CamH][CamW];//存储原灰度图像数组
extern uint8 Reimages[ReCamH][ReCamW];//存储逆透视变换后图像数组
extern uint8 dofhead;//车头距离道路边界 distance of head




typedef struct roadside {//存储八邻域算法结果的结构体
    /*注意图像左上角为坐标原点！*/
    int x;//x坐标
    int y;//y坐标
    double angle;//该点计算角度
}roadside;
typedef struct angelpoint {//存储八邻域算法结果的结构体
    roadside orgpoint;
    int num;//index
    int angletype;//拐点类型,1为下内角，2为上内角，-1为外角
}angelpoint;
typedef struct angellist {//存储八邻域算法结果的结构体
    vector<angelpoint> List;
    vector<angelpoint> typeOneList;//下内角列表
    vector<angelpoint> typeTwoList;//上内角列表
    vector<angelpoint> typeThreeList;//外角列表
    int angelnum;
}angellist;
extern angellist RightAngelList;
extern angellist LeftAngelList;
enum EightWay { l, r };//八临域算法巡线方向枚举量
extern enum EightWay LEFT;//左边巡线使用的标志位
extern enum EightWay RIGHT;//右边巡线使用的标志位
extern int posofleft;//左边八临域巡线使用的开始扫描种子（起始）点相对位置 position of left
extern int posofright;//右边八临域巡线使用的开始扫描种子（起始）点相对位置 position of right
extern roadside leftside[230];//左边线八临域结构体，每次初始化
extern uint8 length_leftside;//左边八临域结构体数组长度,每次初始化
extern roadside rightside[230];//右边线八临域结构体，每次初始化
extern uint8 length_rightside;//右边八临域结构体数组长度,每次初始化
extern uint8 midline[CamH];//拟合出来实际道路中线，每次初始化
extern uint8 midleft[CamH];//无重叠左边边线，每次初始化
extern uint8 validmidleft[CamH];//为了得到无重叠左边边线设置的验证数组,每次初始化
extern uint8 midright[CamH];//无重叠右边边线，每次初始化
extern uint8 validmidright[CamH];//为了得到无重叠右边边线设置的验证数组,每次初始化
extern uint8 reallostleftside;//左边真实丢线数，每次初始化
extern uint8 startlostleftside;//左边开始计逻辑丢线的位置，对应y坐标
extern uint8 lostleftside;//左边丢线数，每次初始化
extern uint8 lostleft_pos_end;//左边丢线结束寻找位置,对应y坐标
extern uint8 reallostrightside;//右边真实丢线数，每次初始化
extern uint8 startlostrightside;//右边开始计逻辑丢线的位置，对应y坐标
extern uint8 lostrightside;//右边丢线数，每次初始化
extern uint8 lostright_pos_end;//右边丢线结束寻找位置,对应y坐标
extern struct roadside cirpos[9];//八邻域寻找下一点时坐标变化查找表
                                                                                            //从12点方向逆时针旋转查找表
//逆透视变换矩阵
extern double change_un_Mat[3][3];
typedef struct TrDetectResult {
    int type;
    int x;
    int y;
    int width;
    int height;
}TrDetectResult;
extern int detectTypeNum[13];      
//下标从0开始，type分别对应
//bomb bridge safety cone crosswalk danger evil block patient prop spy thief tumble
// 0     1      2      3      4       5      6    7     8      9    10   11    12
extern std::vector<TrDetectResult> Tr_result;             
extern std::vector<TrDetectResult> conelist;
extern std::vector<TrDetectResult> rightconelist;
extern std::vector<TrDetectResult> leftconelist;                                                               
/*****************************************************************************
函数功能：标志位、相关数组的初始化函数
参数说明：void
返回说明：void
备注：初始化中将二值化后的初等函数image[]的最低行和图像最左右两行以及封顶行全部初始化为black防止越界
*****************************************************************************/
void InitialiseFlag(void);
/*****************************************************************************
函数功能：确定八邻域左右起点的函数
参数说明：void
返回说明：void
备注：先用传统的两边向中间寻边的方式扫描图像下方三行
*****************************************************************************/
void scanbotton(void);
/*****************************************************************************
函数功能：八邻域函数主程序内部调用模块：根据左右侧指示实现八邻域八个点的次序查询
参数说明：当前判别点的位置序号pos，指明左右侧的标志way
返回说明：返回下一次判别点的位置
备注：
*****************************************************************************/
int Getnextpos(int pos, EightWay way);
/*****************************************************************************
函数功能：八邻域函数的主程序，实现对每一帧图像的八邻域边界扫描
参数说明：当前结构体点的起始搜寻位置pos，当前要搜寻的结构体lc，判别是左右哪边巡线way
返回说明：roadside结构体
备注：重点优化思路，减少重复扫描
*****************************************************************************/
roadside Eightsearch(int pos, roadside lc, EightWay way);
/*****************************************************************************
函数功能：得到道路的边界，巡线主函数
参数说明：void
返回说明：void
备注：结构体数组的长度暂设225，使用时应根据实际摄像头角度增加或者减少
*****************************************************************************/
void getroadside(void);
/*****************************************************************************
函数功能：得到道路边界丢边的情况
参数说明：void
返回说明：void
备注：得到逻辑丢边和真实丢边
*****************************************************************************/                                                                      //求左右丢边情况
void getlostside(void);
/*****************************************************************************
函数功能：对灰度图进行二值化
参数说明：传入大津法所得阈值
返回说明：void
备注：
*****************************************************************************/
void Binaryisation(int yuzhi);
/*******************************************************************************
函数名称：get_min_value
函数功能: 求一列数组中的最小值
输入：a[]—数组    n— 个数
输出：最小值
*******************************************************************************/
uint8 get_min_value(uint8 a[], int n);
/*****************************************************************************
函数功能：寻找封顶行
参数说明：void
返回说明：void
备注：注意此函数必须优先运行，至少在初始化之前运行,并且改变图像宽度之后改部分的搜点个数也要改
*****************************************************************************/
void Find_FengDing(void);
/*****************************************************************************
函数功能：得到车头距离边界的距离
参数说明：void
返回说明：void
备注：
*****************************************************************************/
void getdistance(void);
/*****************************************************************************
函数功能：大津法求阈值
参数说明：*images输入灰度图数组指针，col图像宽度，row图像高度
返回说明：int 该幅灰度图二值化阈值
备注：
*****************************************************************************/                                                                                                                           //大津法求阈值
int otsuThreshold(uint8 *images, int col, int row);
/******************************************************************************
函数功能：获取最终的的实时中线
参数说明：void
返回说明：void
备注：
*****************************************************************************/
// void getmidline(void);
/******************************************************************************
函数功能：逆透视静态测试
参数说明：void
返回说明：void
备注：
*****************************************************************************/
void ImagePerspective_Static();

void searchAngle(angellist &LeftList, angellist &RightList, roadside *LeftBound, roadside *RightBound);

float Get_angle(float Ax, float Ay, float Bx, float By, float Cx, float Cy);

float vector_cos(roadside bottom, roadside mid, roadside top);


enum ElementFlag{Normal,
                PreCross,
                PreRightRound,InRightRound,OutRightRound,AcrossRightRound,
                PreLeftRound,InLeftRound,OutLeftRound,AcrossLeftRound,
                ZebraFlag,
                RecMaintenWay,PreMainten,InMainten,OutMainten,
                PreBoom,YellowHinder};//特殊元素主状态标志位
extern std::string ElementString[17];//特殊元素主状态标志位显示字符串
extern enum ElementFlag elementflag;//主状态主标志位
extern std::string ControSideFlagString[4];
enum ControSideFlag{MidSide,RightSide,LeftSide,Straight};//依赖左右边界巡线标志位
extern enum ControSideFlag controsideflag;//依赖左右边界巡线主标志位
//打角行一定情况下动态，最大不超过75
extern uint8 waycontrolline1;//第一打角行48
extern uint8 waycontrolline2;//第二打角行49
extern uint8 realcontrolline1;//动态打角行1
extern uint8 realcontrolline2;//动态打角行2
extern uint8 Sidepianyi;//左右巡线偏移量

extern bool MakeLineFlag;//补线之后的特殊显示
extern bool OutPreRightRoundFlag;
extern bool OutPreLeftRoundFlag;
extern bool FindLeftCPointCross;
extern bool FindRightCPointCross;
extern bool ZebraWayFlag;//入库方向

/**************************************************************************************************************************/
//十字部分处理
void MakePreCrossLine(void);
// void FindCrossTopChangePoint(void);
// void RecPreCross(void);
/**************************************************************************************************************************/
//识别斑马线部分
void recozebra(void);
/**************************************************************************************************************************/
extern double RightMeank;//右道路边界平均k值
extern double SRightk;//右道路边界k值方差
extern double LeftMeank;//左道路边界平均k值
extern double SLeftk;//左道路边界k值方差
/**************************************************************************************************************************/
//左右道路边界斜率方差与平均斜率计算
void RoadK(int way,uint8 start,uint8 end);
/**************************************************************************************************************************/
bool RecInRoundConer(int way);
//右环岛处理
void RecPreRightRound(void);
void RecInRightRound(void);
void RecOutRightRound(void);
void RecAcrossrightRound(void);
/**************************************************************************************************************************/
//左环岛处理
void RecPreLeftRound(void);
void RecInLeftRound(void);
void RecOutLeftRound(void);
void RecAcrossLeftRound(void);
/**************************************************************************************************************************/
//红色障碍物处理
extern cv::Mat RedHinder;
extern bool Redflag;
extern int Redsencount;
bool RecRed(Mat &src);
/**************************************************************************************************************************/
//黄色锥桶识别
void RecYellowCone(Mat src);
/**************************************************************************************************************************/
//状态复位计数位
extern int32 OutCrossCount;
extern int32 OutPreRightRoundCount;
extern int32 OutOutRightRoundCount;
extern int32 OutAcrossrightRoundCount;
extern int32 OutPreLeftRoundCount;
extern int32 OutOutLeftRoundCount;
extern int32 OutAcrossleftRoundCount;
extern int32 OutPreRound;//入环岛鲁棒性保证
extern int32 OutRoundCount;//出环岛鲁棒性保证
extern int32 OutZebraCount;
extern int SumZebraCount;//一过斑马线而不入
extern int ZebraGuai;
extern bool ZebraEnable;
extern int32 CountZebraEnable;
/**************************************************************************************************************************/
void ReconElements(void);

int Sidepianyifitting(void);

/*
函数功能：获取最终的的实时中线
参数说明：void
返回说明：void
备注：
*/
void getmidline(void);

void getcontroldata(uint8 *sendlist,int &control,int &motor,int SH,int SHR,int SM,int SMR,int SL);

#endif