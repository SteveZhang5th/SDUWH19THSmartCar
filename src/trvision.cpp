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
//#include "MyMath.hpp"
//#include "trvision.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

using namespace cv;

int Threshold = 0;//存储每幅灰度图像计算出的二值化阈值
uint8 toplinearry[14] ={4,4,4,4,4,4,4,4,4,4,4,4,4,4};//寻找有效封顶行所使用的数组，初始化为图像最顶端
uint8 Topline = CamH - 6;//封顶行
uint8 images[CamH][CamW];//存储原灰度图像数组
uint8 Reimages[ReCamH][ReCamW];//存储逆透视变换后图像数组
uint8 dofhead = 0;//车头距离道路边界 distance of head
angellist RightAngelList;
angellist LeftAngelList;
enum EightWay LEFT = l;//左边巡线使用的标志位
enum EightWay RIGHT = r;//右边巡线使用的标志位
int posofleft = 1;//左边八临域巡线使用的开始扫描种子（起始）点相对位置 position of left
int posofright = 1;//右边八临域巡线使用的开始扫描种子（起始）点相对位置 position of right
roadside leftside[350] = { {0},{0} };//左边线八临域结构体，每次初始化
int length_leftside = 0;//左边八临域结构体数组长度,每次初始化
int length_half_leftside = 0;//左边八临域结构体数组第一次到一半图像高度时的长度，每次初始化
roadside rightside[350] = { {0},{0} };//右边线八临域结构体，每次初始化
int length_rightside = 0;//右边八临域结构体数组长度,每次初始化
int length_half_rightside = 0;//右边八临域结构体数组第一次到一半图像高度时的长度，每次初始化
uint8 midline[CamH] = { 0 };//拟合出来实际道路中线，每次初始化
uint8 midleft[CamH] = { 0 };//无重叠左边边线，每次初始化
uint8 validmidleft[CamH] = { 0 };//为了得到无重叠左边边线设置的验证数组,每次初始化
uint8 midright[CamH] = { 0 };//无重叠右边边线，每次初始化
uint8 validmidright[CamH] = { 0 };//为了得到无重叠右边边线设置的验证数组,每次初始化
uint8 reallostleftside = 0;//左边真实丢线数，每次初始化
uint8 crosslostleftside = 0;//左边角落真实丢线数，每次初始化
uint8 startlostleftside = 0;//左边开始计逻辑丢线的位置，对应y坐标
uint8 lostleftside = 0;//左边丢线数，每次初始化
uint8 lostleft_pos_end = 0;//左边丢线结束寻找位置,对应y坐标
uint8 reallostrightside = 0;//右边真实丢线数，每次初始化
uint8 crosslostrightside = 0;//左边角落真实丢线数，每次初始化
uint8 startlostrightside = 0;//右边开始计逻辑丢线的位置，对应y坐标
uint8 lostrightside = 0;//右边丢线数，每次初始化
uint8 lostright_pos_end = 0;//右边丢线结束寻找位置,对应y坐标
struct roadside cirpos[9] = { {0,0},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} };//八邻域寻找下一点时坐标变化查找表
                                                                                            //从12点方向逆时针旋转查找表
//逆透视变换矩阵
double change_un_Mat[3][3] ={{0.815152,-0.428477,16.334382},{-0.000000,0.200802,5.914327},{0.000000,-0.005972,0.824109}};                

int detectTypeNum[10] = {0}; 
//下标从0开始，type分别对应
//bomb bridge safety cone danger evil patient spy thief tumble
// 0     1      2      3    4     5      6     7    8    9 
std::vector<TrDetectResult> Tr_result;             
std::vector<TrDetectResult> conelist;
std::vector<TrDetectResult> rightconelist;
std::vector<TrDetectResult> leftconelist; 
/*****************************************************************************
函数功能：标志位、相关数组的初始化函数
参数说明：void
返回说明：void
备注：初始化中将二值化后的初等函数image[]的最低行和图像最左右两行以及封顶行全部初始化为black防止越界
*****************************************************************************/
void InitialiseFlag(void){
     length_leftside = 0;
     length_half_leftside = 0;
     length_rightside = 0;
     length_half_rightside = 0;
     memset(validmidleft, '\0', CamH);
     memset(validmidright, '\0', CamH);
     for(int i = 0;i < CamH;i++){
          midleft[i] = 0;//0
          midright[i] = CamW - 1;//CamW - 1
          midline[i] = Mid;
     }
     for (int i = 0; i < 345; i++) {
         leftside[i].x = 1;
         rightside[i].x = CamW - 1;
         if(i < 90){
            leftside[i].y = CamH - i;
            rightside[i].y = CamH - i;
         }else{
            leftside[i].y = 1;
            rightside[i].y = 1;
         }
     }

     for(int i = 0;i < CamW - 1;i++){//给图像人为制造黑边
        images[CamH - 1][i] = BLACK;//下边界置黑
        images[Topline][i] = BLACK;//上边界置黑,找封顶行函数应该在这之前运行
        if(i <= CamH - 1){
            images[i][0] = BLACK;//左边界置黑
            images[i][CamW - 1] = BLACK;//右边界置黑
        }
     }

     reallostleftside = 0;
     crosslostleftside = 0;
     startlostleftside = 0;
     lostleftside = 0;
     reallostrightside = 0;
     crosslostrightside = 0;
     startlostrightside = 0;
     lostrightside = 0;
     lostleft_pos_end = 89;
     lostright_pos_end = 89;
}

/*****************************************************************************
函数功能：确定八邻域左右起点的函数
参数说明：void
返回说明：void
备注：先用传统的两边向中间寻边的方式扫描图像下方三行
*****************************************************************************/
void scanbotton(void) {                                                             //底部搜线寻找种子
    uint8 arrange = 0;//计数变量
    uint8 start = Mid;//初始化起点
    
    if (images[CamH - 2][start] == BLACK)  //先找中间，如果是黑的，不对
    {
      start = CamW / 4;
      if (images[CamH - 2][start] == BLACK)   //再找左边，如果是黑的，不对
      {
        start = CamW * 3 / 4;
        if (images[CamH - 2][start] == BLACK)   //再找右边，如果是黑的，不对
        {
          start = CamW / 8;
          if (images[CamH - 2][start] == BLACK)   //再找更左边一点，如果是黑的，不对
          {
            start = CamW * 7/ 8;
          }else{
            start = Mid;
          }
        }
      }
    }

    for (uint8 line = CamH - 2; line > CamH - 5; line--) {//对最底部三行进行扫描确定左右两侧八邻域起始点

        for (arrange = 0; start + arrange < CamW - 1; arrange++) {//向右寻找

            if (images[line][start + arrange] == BLACK
               && images[line][start + arrange + 1] == BLACK//如果扫描到连续黑点
                ) {
                rightside[length_rightside].x = (start + arrange);
                rightside[length_rightside].y = (line);     //将黑白跳变中的黑点作为八邻域起点
                break;
            }
            else if (start + arrange >= CamW - 2) {//如果触碰图像右边界，将边界黑点作为起点
                rightside[length_rightside].x = CamW - 1;
                rightside[length_rightside].y = line;
                break;
            }
        }//以下寻找左起点与上同理

        for (arrange = 0; start - arrange > 0; arrange++) {

            if (images[line][start - arrange] == BLACK
                && images[line][start - arrange - 1] == BLACK
                ) {
                leftside[length_leftside].x = start - arrange;
                leftside[length_leftside].y = line;
                break;
            }
            else  if (start - arrange <= 1) {
                leftside[length_leftside].x = 0;
                leftside[length_leftside].y = line;
                break;
            }

        }
        midleft[line] = leftside[length_leftside].x;
        midright[line] = rightside[length_rightside].x;
        midline[line] = (leftside[length_leftside].x + rightside[length_rightside].x) / 2;//求取该三行的实时中线
        length_leftside++;//求取该三行产生的左右边界长度
        length_rightside++;
    }
}


/*****************************************************************************
函数功能：八邻域函数主程序内部调用模块：根据左右侧指示实现八邻域八个点的次序查询
参数说明：当前判别点的位置序号pos，指明左右侧的标志way
返回说明：返回下一次判别点的位置
备注：
*****************************************************************************/
int Getnextpos(int pos, EightWay way){
    if (way == l) {
        pos--;
        if (pos < 1)
            pos = 8;
    }
    else if (way == r) {
        pos++;
        if (pos > 8)
            pos = 1;
    }
    return pos;
}
/*****************************************************************************
函数功能：八邻域函数的主程序，实现对每一帧图像的八邻域边界扫描
参数说明：当前结构体点的起始搜寻位置pos，当前要搜寻的结构体lc，判别是左右哪边巡线way
返回说明：roadside结构体
备注：重点优化思路，减少重复扫描
*****************************************************************************/
roadside Eightsearch(int pos, roadside lc, EightWay way){                    //八临域主程序
    int x = lc.x;
    int y = lc.y;
    roadside tar = { x,y + 1 };//若没找到责让下一个点为上一个点的正上方的位置
    int postemp = pos;
    if (way == l) {//逆时针转

        for (int i = 0; i < 8; i++) {

            postemp = Getnextpos(postemp,LEFT);            

/*
8 1 2
7 9 3
6 5 4 
4所在点更靠近原点（在左上角），注意坐标方向
所以实际布局应该是这样的
4 5 6
3 9 7
2 1 8
*/
            
            if (images[y + cirpos[postemp].y][x + cirpos[postemp].x] == BLACK) {
                tar.x = x + cirpos[postemp].x;
                tar.y = y + cirpos[postemp].y;
                /*
                简单来说，直接由当前点寻得目标的结束点的位置，
                进行判别，直接推出下一个点的起始扫描位置,并且从该位置起始不会重复扫描上一个点已经扫描过的地方
                */
                if (postemp == 4 || postemp == 5)//左边巡线关键思路优化点
                    posofleft = 7;
                else if (postemp == 6 || postemp == 7)
                    posofleft = 1;
                else if (postemp == 8 || postemp == 1)
                    posofleft = 3;
                else if (postemp == 2 || postemp == 3)
                    posofleft = 5;
                break;
            }
        }
    }
    else if (way == r) {//顺时针转

        for (int i = 0; i < 8; i++) {

            postemp = Getnextpos(postemp, RIGHT);
            
            if (images[y + cirpos[postemp].y][x + cirpos[postemp].x] == BLACK) {
                tar.x = x + cirpos[postemp].x;
                tar.y = y + cirpos[postemp].y;
                if (postemp == 6 || postemp == 5)//右边巡线关键思路优化点
                    posofright = 3;
                else if (postemp == 8 || postemp == 7)
                    posofright = 5;
                else if (postemp == 1 || postemp == 2)
                    posofright = 7;
                else if (postemp == 4 || postemp == 3)
                    posofright = 1;
                break;
            }
        }
    }

      return tar;
}
/*****************************************************************************
函数功能：得到道路的边界，巡线主函数
参数说明：void
返回说明：void
备注：结构体数组的长度暂设225，使用时应根据实际摄像头角度增加或者减少
*****************************************************************************/
void getroadside(void){
    //初始化部分参数
    int leftnowpos = 7;//left now position 局部变量，当前左边八临域相对位置 
    int rightnowpos = 3;//right now position 局部变量，当前右边八临域相对位置 
    int i = length_leftside;//置为rightside也一样
    bool leftflag = true;//左边界巡线使能标志位
    bool rightflag = true;//右边界巡线使能标志位
    bool leftHalfflag = true;
    bool rightHalfflag = true;
   
    for (; i <= 345; i++) {//主循环，左右边界同时使用这个for循环

        if (leftflag) {//左边巡线
            //由上一个结构体点得到当前结构体点
            leftside[i] = Eightsearch(leftnowpos, leftside[i - 1], LEFT);
            
            length_leftside++;

            if(lostleft_pos_end > leftside[i].y)
                lostleft_pos_end = leftside[i].y;

            if(leftside[i].y  > 23 && leftHalfflag){
                length_half_leftside = length_leftside;
            }else if(leftside[i].y  == 23 && leftHalfflag){
                length_half_leftside = length_leftside;
                leftHalfflag = false;
            }
           

            //更新当前八临域相对位置
            leftnowpos = posofleft;

            //判别停止寻左边界
            if (leftside[i].y <= Topline || leftside[i].y < 0 || leftside[i].y > CamH - 1 || leftside[i].x < 0 || leftside[i].x > CamW - 1)
                leftflag = false;
            //如若连续几个点的位置一样，判别为进入未知情况死循环，退出判别
            else if(leftside[i].x == leftside[i - 1].x && leftside[i].y == leftside[i - 1].y &&
                    leftside[i - 1].x == leftside[i - 2].x && leftside[i - 1].y == leftside[i - 2].y &&
                    leftside[i - 2].x == leftside[i - 3].x && leftside[i - 2].y == leftside[i - 3].y &&
                    leftside[i - 3].x == leftside[i - 4].x && leftside[i - 3].y == leftside[i - 4].y&& i > 6)
                    leftflag = false;

            //判断该点是否为传统的一维数组边界
            if (0 == validmidleft[leftside[i].y] ||leftside[i].x > midleft[leftside[i].y]){
                    midleft[leftside[i].y] = leftside[i].x;//设置为赛道左边界
                    validmidleft[leftside[i].y] = 1;
            }        

        }
        if (rightflag) {//右边巡线，以此类推
            
            rightside[i] = Eightsearch(rightnowpos, rightside[i - 1], RIGHT);

            length_rightside++;

            if(lostright_pos_end > rightside[i].y)
                lostright_pos_end = rightside[i].y;

            if(rightside[i].y  > 23 && rightHalfflag){
                length_half_rightside = length_rightside;
            }else if(rightside[i].y  == 23 && rightHalfflag){
                length_half_rightside = length_rightside;
                rightHalfflag = false;
            }

            rightnowpos = posofright;

            if (rightside[i].y <= Topline || rightside[i].y < 0 || rightside[i].y > CamH - 1 || rightside[i].x < 0 || rightside[i].x > CamW - 1)
                rightflag = false;
            else if(rightside[i].x == rightside[i - 1].x && rightside[i].y == rightside[i - 1].y &&
                    rightside[i - 1].x == rightside[i - 2].x && rightside[i - 1].y == rightside[i - 2].y &&
                    rightside[i - 2].x == rightside[i - 3].x && rightside[i - 2].y == rightside[i - 3].y &&
                    rightside[i - 3].x == rightside[i - 4].x && rightside[i - 3].y == rightside[i - 4].y&& i > 6)
                    rightflag = false;
            
            //判断该点是否为传统的一维数组边界
            if (0 == validmidright[rightside[i].y] ||rightside[i].x < midright[rightside[i].y]){
                    midright[rightside[i].y] = rightside[i].x;//设置为赛道右边界
                    validmidright[rightside[i].y] = 1;
            }  
        }

        if (!leftflag && !rightflag){//如果左右两边均已完成搜线，则退出该for循环
            for(int i = lostleft_pos_end; i > 0;i--){
                 midleft[i] = midleft[lostleft_pos_end];
            }
            for(int i = lostright_pos_end; i > 0;i--){
                 midright[i] = midright[lostright_pos_end];
            }
            break;
        }
    }

}
/*****************************************************************************
函数功能：得到道路边界丢边的情况
参数说明：void
返回说明：void
备注：得到逻辑丢边和真实丢边
*****************************************************************************/                                                                      //求左右丢边情况
void getlostside(void){
    bool enableright = false;
    bool enableleft = false;
    for(int i = CamH - 5;i > 18;i--){

        if(i > lostleft_pos_end){

            if(midleft[i] < 3 && i > 70)
                crosslostleftside++;

            if(midleft[i] < 3)
                reallostleftside++;

            if(midright[i] > CamW - 4 && i > 70)
                crosslostrightside++;

            if(midright[i] > CamW - 4)
                reallostrightside++;
        }

        if(!enableleft && i > lostleft_pos_end){
            if(midleft[i] < 3 && midleft[i + 1] >= 3){
                lostleftside++;
                enableleft = true;

                if(startlostleftside == 0)
                    startlostleftside = i;
                
            }
        }else if(i > lostleft_pos_end){
            if(midleft[i] < 3){
                lostleftside++;
            }else{
                enableleft = false; 
            }
        }

        if(!enableright && i > lostright_pos_end){
            if(midright[i] > CamW - 4 && midright[i + 1] <= CamW - 4){
                lostrightside++;
                enableright = true;

                if(startlostrightside == 0)
                    startlostrightside = i;
                
            }
        }else if(i > lostright_pos_end){
            if(midright[i] > CamW - 4){
                lostrightside++;
            }else{
                enableright = false; 
            }
        } 

    }
    searchAngle(LeftAngelList,RightAngelList,leftside,rightside);


}
/*****************************************************************************
函数功能：对灰度图进行二值化
参数说明：传入大津法所得阈值
返回说明：void
备注：
*****************************************************************************/
void Binaryisation(int yuzhi) {                                  
    for (int i = 0; i < CamH; i++) {
        for (int j = 0; j < CamW; j++) {
            if (images[i][j] > yuzhi ) {
                images[i][j] = WHITE;  
            }
            else {
                images[i][j] = BLACK;
            }
        }
    }
}
/*******************************************************************************
函数名称：get_min_value
函数功能: 求一列数组中的最小值
输入：a[]—数组    n— 个数
输出：最小值
*******************************************************************************/
uint8 get_min_value(uint8 a[], int n)
{
    uint8 i, min;
    min = a[0];
    for (i = 1; i < n; i++)
    {
        if (min > a[i])
            min = a[i];
    }
    return min;
}
/*****************************************************************************
函数功能：寻找封顶行
参数说明：void
返回说明：void
备注：注意此函数必须优先运行，至少在初始化之前运行,并且改变图像宽度之后改部分的搜点个数也要改
*****************************************************************************/
void Find_FengDing(void)
{
    uint8 i,j;
    
    Topline=0;

    for(j = 9;j <= CamW - 11;j += 10)//找了14个点
    {
        for(i = CamH-2;i >= 4;i--)//
        {
            if(images[i][j] == BLACK){
                toplinearry[(j+1)/10 - 1] = i;
                break;
            }else 
                toplinearry[(j+1)/10 - 1] = CamH-2;

            if(i == 4){//出现最小封顶，直接封顶并跳出循环
                toplinearry[(j+1)/10 - 1] = 4;
                break;
            }
        }

        if(toplinearry[(j+1)/10 - 1] == 4)//出现最小封顶，直接封顶并跳出循环
            break;

    }
    //---比较出最小封顶点，作为最终封顶点fd_car,即这个封顶为最远点
    Topline = get_min_value(toplinearry ,14);
}
/*****************************************************************************
函数功能：得到车头距离边界的距离
参数说明：void
返回说明：void
备注：
*****************************************************************************/
void getdistance(void){
    dofhead = 0;
    for ( uint8 line = CamH - 3; line > 2; line--) {
        if(images[line][Mid] == WHITE &&images[line - 1][Mid] == BLACK &&images[line - 2][Mid] == BLACK)
            break;
        else if(images[line][Mid] == BLACK &&images[line - 1][Mid] == BLACK &&images[line - 2][Mid] == BLACK && line  == CamH - 4){
            break;
        }else{
            dofhead++;
        }
    }
}
/*****************************************************************************
函数功能：大津法求阈值
参数说明：*images输入灰度图数组指针，col图像宽度，row图像高度
返回说明：int 该幅灰度图二值化阈值
备注：
*****************************************************************************/                                                                                                                           //大津法求阈值
int otsuThreshold(uint8 *images, int col, int row) {//
#define GrayScale 256
    int width = col;
    int height = row;
    int pixelCount[GrayScale] = {0};
    float pixelPro[GrayScale] = {0};
    int i, j, pixelSum = width * height / 4;
    int threshold = 0;
    uint8* data = images;  //指向像素数据的指针
    
    int gray_sum = 0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i += 2)//改过，截出了一部分高度的图像
    {
        for (j = 0; j < width; j += 2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum += (int)data[i * width + j];       //灰度值总和
        }
    }
    //计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;

    }
    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 5; j <= 250; j++)//优化
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (int)j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    return threshold;
}
/******************************************************************************
函数功能：获取最终的的实时中线
参数说明：void
返回说明：void
备注：
*****************************************************************************/
// void getmidline(void){                                
//     for (int i = CamH - 4; i > Topline; i--) 
//         midline[i] = (midleft[i] + midright[i])/2;
// }

/******************************************************************************
函数功能：逆透视静态测试
参数说明：void
返回说明：void
备注：
*****************************************************************************/
void ImagePerspective_Static(){

    for (int i = 0; i < ReCamW ;i++) {
        for (int j = 0; j < ReCamH ;j++) {
            int local_x = (int) ((change_un_Mat[0][0] * i
                    + change_un_Mat[0][1] * j + change_un_Mat[0][2])
                    / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j
                            + change_un_Mat[2][2]));
            int local_y = (int) ((change_un_Mat[1][0] * i
                    + change_un_Mat[1][1] * j + change_un_Mat[1][2])
                    / (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j
                            + change_un_Mat[2][2]));
            if (local_x >= 0&& local_y >= 0 && local_y < CamH && local_x < CamW){
                Reimages[j][i] = images[local_y][local_x];
            }
            else {
                Reimages[j][i] = BLACK;          //&PER_IMG[0][0];
            }
        }   
    }
}

void searchAngle(angellist &LeftList, angellist &RightList, roadside *LeftBound, roadside *RightBound){
    int i = 0;
    int searchmid;
    int searchtop;
    int searchbottom;

    float cossita;
    float arcos_now = 0;

    LeftList.List.clear();
    LeftList.typeOneList.clear();
    LeftList.typeTwoList.clear();
    LeftList.typeThreeList.clear();
    LeftList.angelnum = 0;

    for (i = searchbottom = 0, searchmid = searchbottom + 7, searchtop = searchmid + 7; searchtop < length_leftside; searchbottom+=4, searchmid+=4, searchtop+=4)
    {
        if(LeftBound[searchmid].y<85)
        {   

            cossita = vector_cos(LeftBound[searchbottom], LeftBound[searchmid], LeftBound[searchtop]) * 1.0f;

            cossita = func_limit(cossita, 1);
            
            arcos_now = acosf(cossita) * 180.0f / pi;

            LeftBound[searchmid].angle = arcos_now;
            // LeftBound[searchmid].angle = Get_angle(LeftBound[searchbottom].x,LeftBound[searchbottom].y,
            //                                         LeftBound[searchmid].x,LeftBound[searchmid].y,
            //                                         LeftBound[searchtop].x,LeftBound[searchtop].y);

        }
    }

    for (i = 10; i + 5 < length_leftside; i++)
    {
        if (LeftBound[i].angle > 25)
            if (LeftBound[i].angle > LeftBound[i - 1].angle && LeftBound[i].angle > LeftBound[i + 1].angle)          
                if (LeftBound[i].angle > LeftBound[i - 2].angle && LeftBound[i].angle > LeftBound[i + 2].angle)
                    if (LeftBound[i].angle > LeftBound[i - 3].angle && LeftBound[i].angle > LeftBound[i + 3].angle)
                        if (LeftBound[i].angle > LeftBound[i - 4].angle && LeftBound[i].angle > LeftBound[i + 4].angle)
                        {
                            angelpoint temp;
                            temp.orgpoint.angle = LeftBound[i].angle;
                            temp.orgpoint.x = LeftBound[i].x;
                            temp.orgpoint.y = LeftBound[i].y;
                            temp.num = i;
                            double midx = ((double)LeftBound[i - 4].x + (double)LeftBound[i + 4].x)/2;
                            double midy = ((double)LeftBound[i - 4].y + (double)LeftBound[i + 4].y)/2;
                            if(midx < (double)temp.orgpoint.x && midy >= (double)temp.orgpoint.y){
                                temp.angletype = 1;
                                LeftList.typeOneList.push_back(temp);
                            }else if(midx <= (double)temp.orgpoint.x && midy < (double)temp.orgpoint.y){   
                                temp.angletype = 2;
                                LeftList.typeTwoList.push_back(temp);
                            }else{
                                temp.angletype = -1;
                                LeftList.typeThreeList.push_back(temp);
                            }

                            LeftList.List.push_back(temp);
                        }
    }
    LeftList.angelnum = LeftList.List.size();

    RightList.List.clear();
    RightList.typeOneList.clear();
    RightList.typeTwoList.clear();
    RightList.typeThreeList.clear();
    RightList.angelnum = 0;

    for (i = searchbottom = 0, searchmid = searchbottom + 7, searchtop = searchmid + 7; searchtop < length_rightside; searchbottom+=4, searchmid+=4, searchtop+=4)
    {
        if(RightBound[searchmid].y<85)
        {
            cossita = vector_cos(RightBound[searchbottom], RightBound[searchmid], RightBound[searchtop]) * 1.0f;

            cossita = func_limit(cossita, 1);

            arcos_now = acosf(cossita) * 180.0f / pi;

            RightBound[searchmid].angle = arcos_now;
            // RightBound[searchmid].angle = Get_angle(RightBound[searchbottom].x,RightBound[searchbottom].y,
            //                                         RightBound[searchmid].x,RightBound[searchmid].y,
            //                                         RightBound[searchtop].x,RightBound[searchtop].y);

        }
    }

    for (i = 10; i + 5 < length_rightside; i++)
    {
        if (RightBound[i].angle > 25)
            if (RightBound[i].angle > RightBound[i - 1].angle && RightBound[i].angle > RightBound[i + 1].angle)
                if (RightBound[i].angle > RightBound[i - 2].angle && RightBound[i].angle > RightBound[i + 2].angle)
                    if (RightBound[i].angle > RightBound[i - 3].angle && RightBound[i].angle > RightBound[i + 3].angle)
                        if (RightBound[i].angle > RightBound[i - 4].angle && RightBound[i].angle > RightBound[i + 4].angle)
                        {
                            angelpoint temp;
                            temp.orgpoint.angle = RightBound[i].angle;
                            temp.orgpoint.x = RightBound[i].x;
                            temp.orgpoint.y = RightBound[i].y;
                            temp.num = i;
                            double midx = ((double)RightBound[i - 4].x + (double)RightBound[i + 4].x)/2;
                            double midy = ((double)RightBound[i - 4].y + (double)RightBound[i + 4].y)/2;
                            if(midx > (double)temp.orgpoint.x && midy >= (double)temp.orgpoint.y){   
                                temp.angletype = 1;
                                RightList.typeOneList.push_back(temp);
                            }else if(midx >= (double)temp.orgpoint.x && midy < (double)temp.orgpoint.y){
                                temp.angletype = 2;
                                RightList.typeTwoList.push_back(temp);
                            }else{
                                temp.angletype = -1;
                                RightList.typeThreeList.push_back(temp);
                            }

                            RightList.List.push_back(temp);
                        }

    }
    RightList.angelnum = RightList.List.size();
}

float Get_angle(float Ax, float Ay, float Bx, float By, float Cx, float Cy)
{
 
    float BA = 0.00;//向量BA的模
    float BC = 0.00;
    float SBA_BC = 0.00;//向量点乘的值
    float angle = 0.00;
 
    float AX=((change_un_Mat[0][0] * Ax + change_un_Mat[0][1] * Ay + change_un_Mat[0][2])/(change_un_Mat[2][0] * Ax + change_un_Mat[2][1] * Ay + change_un_Mat[2][2]));
    float AY=((change_un_Mat[1][0] * Ax + change_un_Mat[1][1] * Ay + change_un_Mat[1][2])/(change_un_Mat[2][0] * Ax + change_un_Mat[2][1] * Ay + change_un_Mat[2][2]));
    float BX=((change_un_Mat[0][0] * Bx + change_un_Mat[0][1] * By + change_un_Mat[0][2])/(change_un_Mat[2][0] * Bx + change_un_Mat[2][1] * By + change_un_Mat[2][2]));
    float BY=((change_un_Mat[1][0] * Bx + change_un_Mat[1][1] * By + change_un_Mat[1][2])/(change_un_Mat[2][0] * Bx + change_un_Mat[2][1] * By + change_un_Mat[2][2]));
    float CX=((change_un_Mat[0][0] * Cx + change_un_Mat[0][1] * Cy + change_un_Mat[0][2])/(change_un_Mat[2][0] * Cx + change_un_Mat[2][1] * Cy + change_un_Mat[2][2]));
    float CY=((change_un_Mat[1][0] * Cx + change_un_Mat[1][1] * Cy + change_un_Mat[1][2])/(change_un_Mat[2][0] * Cx + change_un_Mat[2][1] * Cy + change_un_Mat[2][2]));
 
    BA = sqrt((AX-BX)*(AX-BX)+(AY-BY)*(AY-BY));
    BC = sqrt((CX-BX)*(CX-BX)+(CY-BY)*(CY-BY));
 
    SBA_BC = (AX-BX)*(CX-BX)+(AY-BY)*(CY-BY);
 
    angle =  acos(SBA_BC*1.00/(BA*BC));
 
    return angle*57.3;
}

float vector_cos(roadside bottom, roadside mid, roadside top){

    float difftopx = (float)(top.x - mid.x);
    float difftopy = (float)(top.y - mid.y);
    float diffmidx = (float)(mid.x - bottom.x);
    float diffmidy = (float)(mid.y - bottom.y);

    return (difftopx * diffmidx + difftopy * diffmidy) / (sqrtf(difftopx * difftopx + difftopy * difftopy) * sqrtf(diffmidx * diffmidx + diffmidy * diffmidy));

}


std::string ElementString[23] = {
    "Normal",
    "PreRightRound","InRightRound","OutRightRound","AcrossRightRound",
    "PreLeftRound","InLeftRound","OutLeftRound","AcrossLeftRound",
    "ZebraFlag",
    "RecMaintenWay","PreMainten","InMainten","OutMainten",
    "PreBoom","YellowHinder","OutHinder",
    "GoSafety",
    "PreDanger",
    "PreSpy","OverSpy","StopSpy",
    "Bridge"};//特殊元素主状态标志位显示字符串
enum ElementFlag elementflag = Normal;//主状态主标志位
extern enum CrossFlag crossflag = None;//主状态主标志位
std::string ControSideFlagString[4] = {"MidSide","RightSide","LeftSide","Straight"};
enum ControSideFlag controsideflag = MidSide;//依赖左右边界巡线主标志位
//打角行一定情况下动态，最大不超过75
uint8 waycontrolline1 = CONTROLLINEONE;//第一打角行40
uint8 waycontrolline2 = CONTROLLINETWO;//第二打角行41
uint8 realcontrolline1 = CONTROLLINEONE;//动态打角行40
uint8 realcontrolline2 = CONTROLLINETWO;//动态打角行41
uint8 Sidepianyi = 38;//左右巡线偏移量

int Track_width[CamH] = {19, 20, 21, 22, 23,  24,  25,  26,  27,  28,   // 0-9预测
                              29, 30, 31, 33, 34,  35,  37,  38,  39,  41,   // 10-19预测（真实）
                              43, 43, 45, 47, 48,  49,  51,  52,  53,  55,   // 20-29真实（真实）
                              56, 58, 59, 60, 62,  63,  64,  66,  67,  69,   // 30-39真实（真实）
                              70, 71, 73, 74, 75,  76,  78,  79,  80,  82,   // 40-49预测(真实)
                              83, 84, 86, 87, 88,  90,  90,  92,  93,  94,   // 50-59预测（真实）
                              96, 96, 98, 99, 100, 102, 103, 104, 105, 106}; // 60-69预测（真实）

bool MakeLineFlag = true;//补线之后的特殊显示
bool OutPreRightRoundFlag = false;
bool OutPreLeftRoundFlag = false;
bool FindLeftCPointCross = false;
bool FindRightCPointCross = false;
bool ZebraWayFlag = false;//入库方向,false向右拐
int CrossSumCount = 0;
int CrossInCount = 0;
int CrossOutCount = 0;
int InAlphaCount = 0;
/**************************************************************************************************************************/
//十字部分处理
void MakePreCrossLine(void){

    if(RightAngelList.angelnum == 0 || LeftAngelList.angelnum == 0)
        return;
    
    if(lostleftside > 10 &&lostrightside > 10){
        //正常四个点全部找到
        if(!RightAngelList.typeOneList.empty() &&!RightAngelList.typeTwoList.empty() &&RightAngelList.typeThreeList.size() >= 1
         &&!LeftAngelList.typeOneList.empty() &&!LeftAngelList.typeTwoList.empty() &&LeftAngelList.typeThreeList.size() >= 1){

            int x1 = RightAngelList.typeOneList[0].orgpoint.x;
            int y1 = RightAngelList.typeOneList[0].orgpoint.y;
            int x2 = RightAngelList.typeTwoList[0].orgpoint.x;//上内角
            int y2 = RightAngelList.typeTwoList[0].orgpoint.y;
            
            int rightstart = y1;
            int rightend = y2;

            if(x1 < x2){//只用right的下角来拟合
                if(RightAngelList.typeOneList[0].num - 6 <= 0)
                    return;
                x1 = rightside[RightAngelList.typeOneList[0].num - 2].x;
                y1 = rightside[RightAngelList.typeOneList[0].num - 2].y;
                x2 = rightside[RightAngelList.typeOneList[0].num - 6].x;
                y2 = rightside[RightAngelList.typeOneList[0].num - 6].y;
                rightstart = y1;
                rightend = 2;
            }

            float k_right = myk((float)x1,(float)y1,(float)x2,(float)y2);
            float b_right = myb((float)x1,(float)y1,(float)x2,(float)y2);
            for(int i = rightstart;i >= rightend;i--){
                int temp = ((float)i - b_right)/k_right;
                if(temp < 0)
                    temp = 0;
                else if(temp > CamW - 1)
                    temp = CamW - 1;

                midright[i] = (uint8)temp;
            }

            x1 = LeftAngelList.typeOneList[0].orgpoint.x;
            y1 = LeftAngelList.typeOneList[0].orgpoint.y;
            x2 = LeftAngelList.typeTwoList[0].orgpoint.x;
            y2 = LeftAngelList.typeTwoList[0].orgpoint.y;

            int leftstart = y1;
            int leftend = y2;

            if(x1 > x2){
                if(LeftAngelList.typeOneList[0].num - 6 <= 0)
                    return;
                x1 = leftside[LeftAngelList.typeOneList[0].num - 2].x;
                y1 = leftside[LeftAngelList.typeOneList[0].num - 2].y;
                x2 = leftside[LeftAngelList.typeOneList[0].num - 6].x;
                y2 = leftside[LeftAngelList.typeOneList[0].num - 6].y;
                leftstart = y1;
                leftend = 2;
            }
                

            float k_left = myk((float)x1,(float)y1,(float)x2,(float)y2);
            float b_left = myb((float)x1,(float)y1,(float)x2,(float)y2);
            for(int i = leftstart;i >= leftend;i--){
                int temp = ((float)i - b_left)/k_left;
                if(temp < 0)
                    temp = 0;
                else if(temp > CamW - 1)
                    temp = CamW - 1;

                midleft[i] = (uint8)temp;
            }
            if(crossflag == None){
                CrossInCount++;
            }
            return;
        }
    }

    if(lostrightside > 15){
        //右边上下两点补线，左边只找到左下角的点
        if(!RightAngelList.typeOneList.empty() && !LeftAngelList.typeOneList.empty() 
        && !RightAngelList.typeTwoList.empty()&& !RightAngelList.typeThreeList.empty()){
            int x1 = RightAngelList.typeOneList[0].orgpoint.x;
            int y1 = RightAngelList.typeOneList[0].orgpoint.y;
            int x2 = RightAngelList.typeTwoList[0].orgpoint.x;
            int y2 = RightAngelList.typeTwoList[0].orgpoint.y;
            int rightstart = y1;
            int rightend = y2;
            float k_right = myk((float)x1,(float)y1,(float)x2,(float)y2);
            float b_right = myb((float)x1,(float)y1,(float)x2,(float)y2);
            for(int i = rightstart;i >= rightend;i--){
                int temp = ((float)i - b_right)/k_right;
                if(temp < 0)
                    temp = 0;
                else if(temp > CamW - 1)
                    temp = CamW - 1;

                midright[i] = (uint8)temp;
            }
            if(LeftAngelList.typeOneList[0].num - 6 <= 0)
                return;
            x1 = leftside[LeftAngelList.typeOneList[0].num - 2].x;
            y1 = leftside[LeftAngelList.typeOneList[0].num - 2].y;
            x2 = leftside[LeftAngelList.typeOneList[0].num - 6].x;
            y2 = leftside[LeftAngelList.typeOneList[0].num - 6].y;
            int leftstart = y1;
            int leftend = 2;
            float k_left = myk((float)x1,(float)y1,(float)x2,(float)y2);
            float b_left = myb((float)x1,(float)y1,(float)x2,(float)y2);
            for(int i = leftstart;i >= leftend;i--){
                int temp = ((float)i - b_left)/k_left;
                if(temp < 0)
                    temp = 0;
                else if(temp > CamW - 1)
                    temp = CamW - 1;

                midleft[i] = (uint8)temp;
            }
            if(crossflag == None){
                CrossInCount++;
            }
            return;
        }
    }
    if(lostleftside > 15){
        //左边两个点都找到补线，右边只找到下面的点补线
        if(!RightAngelList.typeOneList.empty() && !LeftAngelList.typeOneList.empty()
            && !LeftAngelList.typeThreeList.empty() && !LeftAngelList.typeTwoList.empty()){
            if(RightAngelList.typeOneList[0].num - 6 <= 0)
                return;

            int x1 = rightside[RightAngelList.typeOneList[0].num - 2].x;
            int y1 = rightside[RightAngelList.typeOneList[0].num - 2].y;
            int x2 = rightside[RightAngelList.typeOneList[0].num - 6].x;
            int y2 = rightside[RightAngelList.typeOneList[0].num - 6].y;
            int rightstart = y1;
            int rightend = 2;
            float k_right = myk((float)x1,(float)y1,(float)x2,(float)y2);
            float b_right = myb((float)x1,(float)y1,(float)x2,(float)y2);
            for(int i = rightstart;i >= rightend;i--){
                int temp = ((float)i - b_right)/k_right;
                if(temp < 0)
                    temp = 0;
                else if(temp > CamW - 1)
                    temp = CamW - 1;

                midright[i] = (uint8)temp;
            }

            x1 = LeftAngelList.typeOneList[0].orgpoint.x;
            y1 = LeftAngelList.typeOneList[0].orgpoint.y;
            x2 = LeftAngelList.typeTwoList[0].orgpoint.x;
            y2 = LeftAngelList.typeTwoList[0].orgpoint.y;
            int leftstart = y1;
            int leftend = y2;
            float k_left = myk((float)x1,(float)y1,(float)x2,(float)y2);
            float b_left = myb((float)x1,(float)y1,(float)x2,(float)y2);
            for(int i = leftstart;i >= leftend;i--){
                int temp = ((float)i - b_left)/k_left;
                if(temp < 0)
                    temp = 0;
                else if(temp > CamW - 1)
                    temp = CamW - 1;

                midleft[i] = (uint8)temp;
            }
            if(crossflag == None){
                CrossInCount++;
            }
            return;
        }
    }

    if(reallostleftside > 15 &&reallostrightside > 15){//只找到了上面两个点
        if((RightAngelList.typeOneList.empty() || RightAngelList.typeOneList[0].orgpoint.y<45) &&!RightAngelList.typeTwoList.empty()
         &&(LeftAngelList.typeOneList.empty() || LeftAngelList.typeOneList[0].orgpoint.y<45) &&!LeftAngelList.typeTwoList.empty()){
            
            //if(RightAngelList.typeTwoList[0].orgpoint.y > waycontrolline2 && LeftAngelList.typeTwoList[0].orgpoint.y > waycontrolline2)
            //  return;//打角行都出去了，没必要补线了


            int x1 = rightside[RightAngelList.typeTwoList[0].num + 2].x;
            int y1 = rightside[RightAngelList.typeTwoList[0].num + 2].y;
            if(RightAngelList.typeTwoList[0].num + 6 > length_rightside)
                return;
            int x2 = rightside[RightAngelList.typeTwoList[0].num + 6].x;
            int y2 = rightside[RightAngelList.typeTwoList[0].num + 6].y;
            int rightstart = CamH - 2;
            int rightend = y2;
            float k_right = myk((float)x1,(float)y1,(float)x2,(float)y2);
            float b_right = myb((float)x1,(float)y1,(float)x2,(float)y2);
            for(int i = rightstart;i >= rightend;i--){
                int temp = ((float)i - b_right)/k_right;
                if(temp < 0)
                    temp = 0;
                else if(temp > CamW - 1)
                    temp = CamW - 1;

                midright[i] = (uint8)temp;

            }

            x1 = leftside[LeftAngelList.typeTwoList[0].num + 2].x;
            y1 = leftside[LeftAngelList.typeTwoList[0].num + 2].y;
            if(LeftAngelList.typeTwoList[0].num + 6 > length_leftside)
                return;
            x2 = leftside[LeftAngelList.typeTwoList[0].num + 6].x;
            y2 = leftside[LeftAngelList.typeTwoList[0].num + 6].y;
            int leftstart = CamH - 2;
            int leftend = y2;
            float k_left = myk((float)x1,(float)y1,(float)x2,(float)y2);
            float b_left = myb((float)x1,(float)y1,(float)x2,(float)y2);
            for(int i = leftstart;i >= leftend;i--){
                int temp = ((float)i - b_left)/k_left;
                if(temp < 0)
                    temp = 0;
                else if(temp > CamW - 1)
                    temp = CamW - 1;

                midleft[i] = (uint8)temp;
            }
            if(crossflag == None){
                CrossInCount++;
            }
            return;
        }
    }

}
/**************************************************************************************************************************/
//识别斑马线部分
void recozebra(void){
    // if(lostleftside > 10 && lostrightside > 10)
    //     return;

    int colorcount = 0;
    int sumcount = 0;
    for(uint8 line = 40; line <= 75;line++){

        for(int a = 125;a >= 25;a--){
            if((images[line][a] == BLACK && images[line][a - 1] == WHITE && images[line][a - 2] == WHITE) 
            || (images[line][a] == WHITE && images[line][a - 1] == BLACK && images[line][a - 2] == BLACK)){
                colorcount++;
            }
            if(colorcount == 12){
                sumcount++;
                colorcount = 0;
                break;
            }
        }

        colorcount = 0;
        if(sumcount == 2){
            elementflag = ZebraFlag;
            cout<<"a ZebraFlag"<<endl;
            if(reallostrightside > 14)
                ZebraWayFlag = false;
            else if(reallostleftside > 14)
                ZebraWayFlag = true;
            break;
        }
    }
}
/**************************************************************************************************************************/
double RightMeank = 0;//右道路边界平均k值
double SRightk = -1;//右道路边界k值方差
double LeftMeank = 0;//左道路边界平均k值
double SLeftk = -1;//左道路边界k值方差
/**************************************************************************************************************************/
//左右道路边界斜率方差与平均斜率计算
void RoadK(int way,uint8 start,uint8 end){//start大于end,注意y坐标方向,预设90到40
    double Sumk = 0;
    std::vector<double> k;
    if(way == 0){//计算右边的斜率情况

        RightMeank = 233;
        SRightk = -1;

        if(end < lostright_pos_end)
            return;
        
        int i = 0;
        double tempk = 0;
        for(i = start;i - 5 >= end;i -= 2){
            tempk = myk(midright[i],i,midright[i - 5],i - 5);
            k.push_back(tempk);
            Sumk += tempk;
        }
        RightMeank = Sumk / i;
        for(uint8 j = 0;j < k.size();j++){
            SRightk += (RightMeank - k[j])*(RightMeank - k[j]);
        }
        SRightk = SRightk / k.size();

        if(RightMeank > 1023){
            RightMeank = 1023;
        }else if(RightMeank < -1023){
            RightMeank = -1023;
        }

        if(SRightk > 514){
            SRightk = 514;
        }else if(SRightk < -514){
            SRightk = -514;
        }

    }else if(way == 1){//计算左边的斜率情况

        LeftMeank = 233;
        SLeftk = -1;

        if(end < lostleft_pos_end)
            return;
        
        int i = 0;
        double tempk = 0;
        for(i = start;i - 5 >= end;i -= 2){
            tempk = myk(midleft[i],i,midleft[i - 5],i - 5);
            k.push_back(tempk);
            Sumk += tempk;
        }
        LeftMeank = Sumk / i;
        for(uint8 j = 0;j < k.size();j++){
            SLeftk += (LeftMeank - k[j])*(LeftMeank - k[j]);
        }
        SLeftk = SLeftk / k.size();

        if(LeftMeank > 1023){
            LeftMeank = 1023;
        }else if(LeftMeank < -1023){
            LeftMeank = -1023;
        }

        if(SLeftk > 514){
            SLeftk = 514;
        }else if(SLeftk < -514){
            SLeftk = -514;
        }

    }

}
/**************************************************************************************************************************/
bool RecInRoundConer(int way){
    int count = 0;
    //int countblack = 0;
    bool result = false;
    if(way == 0){//识别进右下环岛的特征
        for(int i = startlostrightside;i <= CamH - 2;i++){
            if(midright[i] > midright[i + 1] && midright[i] < CamW - 2)
                count++;
            if(count == 5){
                result = true;
                break;
            }
        }
    }else if(way == 1){//识别进左下环岛的特征
        for(int i = startlostleftside;i <= CamH - 2;i++){
            if(midleft[i] < midleft[i + 1]&& midleft[i] > 1)
                count++;
            if(count == 5){
                result = true;
                break;
            }
        }
    }
    return result;
}
//右环岛处理
void RecPreRightRound(void){
    if(lostleftside <= 4 && reallostleftside <= 38 && lostrightside > 18 && lostleft_pos_end < 25 && length_rightside > 120 && length_half_leftside <= 67
    && startlostleftside < 27&&dofhead > 60){
        if(RightAngelList.typeOneList.size()>0 &&RightAngelList.typeOneList[0].orgpoint.angle>=90
        &&RightAngelList.typeThreeList.size()>0
        &&(LeftAngelList.typeOneList.size() == 0 || LeftAngelList.typeOneList[0].orgpoint.y<20)){
            elementflag = PreRightRound;
            cout<<"turn into PreRightRound!"<<endl;
        }
    }
}
void RecInRightRound(void){
    if(lostleftside <= 4 &&lostrightside > 12 &&length_rightside > 120 
    &&RecInRoundConer(0) &&dofhead > 60){
        if(RightAngelList.typeTwoList.size()>0 &&RightAngelList.typeTwoList[0].orgpoint.y<45 
        &&RightAngelList.typeTwoList[0].orgpoint.angle<90 &&RightAngelList.typeThreeList.size()>0 
        &&(LeftAngelList.typeOneList.size() == 0 || LeftAngelList.typeOneList[0].orgpoint.y<30)){
            OutPreRightRoundFlag = true;
        }
    }
}
void RecOutRightRound(void){
    if(reallostrightside >= 20 &&dofhead > 9){
        if(LeftAngelList.typeOneList.size()>0 &&LeftAngelList.typeOneList[0].orgpoint.angle>90){
            elementflag = OutRightRound;
            cout<<"turn into OutRightRound!"<<endl;
        }
    }
}
void RecAcrossrightRound(void){
    if(lostleftside <= 4 && reallostrightside >= 12 && dofhead > 45){
        if(RightAngelList.typeTwoList.size()>0 &&RightAngelList.typeTwoList[0].orgpoint.y<60 
        &&RightAngelList.typeTwoList[0].orgpoint.angle<90 &&RightAngelList.typeThreeList.size()>0 
        &&(LeftAngelList.typeOneList.size() == 0 || LeftAngelList.typeOneList[0].orgpoint.y<30)){
            elementflag = AcrossRightRound;
            cout<<"turn into AcrossRightRound!"<<endl;
        }
    }
    // if(lostleftside <= 4 && reallostrightside >= 12 && dofhead >= 79){
    //     if(RightAngelList.List.size() >= 2){
    //         elementflag = AcrossRightRound;
    //         cout<<"turn into AcrossRightRound!"<<endl;
    //     }
    // }

}
/**************************************************************************************************************************/
//左环岛处理
void RecPreLeftRound(void){
    if(lostleftside >= 12 &&lostrightside <= 1 && reallostrightside <= 38 &&length_leftside > 100 &&length_half_rightside <= 67&&lostright_pos_end < 25
    &&startlostrightside < 27 &&dofhead >= 57){
        if(LeftAngelList.typeOneList.size()>0 &&LeftAngelList.typeOneList[0].orgpoint.angle>=90
        &&LeftAngelList.typeThreeList.size()>0
        &&(RightAngelList.typeOneList.size() == 0 || RightAngelList.typeOneList[0].orgpoint.y<25)){
            elementflag = PreLeftRound;
            cout<<"turn into PreLeftRound!"<<endl;
        }
    }
}
void RecInLeftRound(void){
    if(lostleftside > 12 &&lostrightside <= 4 &&length_leftside > 120 &&RecInRoundConer(1) &&dofhead > 60){
        if(LeftAngelList.typeTwoList.size()>0 &&LeftAngelList.typeTwoList[0].orgpoint.y<45 
        &&LeftAngelList.typeTwoList[0].orgpoint.angle<90 &&LeftAngelList.typeThreeList.size()>0 
        &&(RightAngelList.typeOneList.size() == 0 || RightAngelList.typeOneList[0].orgpoint.y<30)){
            OutPreLeftRoundFlag = true;
        }
    }
}
void RecOutLeftRound(void){
    if(reallostleftside >= 20 &&dofhead > 10){
        if(RightAngelList.typeOneList.size()>0 &&RightAngelList.typeOneList[0].orgpoint.angle>90){
            elementflag = OutLeftRound;
            cout<<"turn into OutLeftRound!"<<endl;
        }
    }
}
void RecAcrossLeftRound(void){
    if(reallostleftside >= 12 && lostrightside <= 4 && dofhead >= 45){
        if(LeftAngelList.typeTwoList.size()>0 &&LeftAngelList.typeTwoList[0].orgpoint.y<60 
        &&LeftAngelList.typeTwoList[0].orgpoint.angle<120 &&LeftAngelList.typeThreeList.size()>0 
        &&(RightAngelList.typeOneList.size() == 0 || RightAngelList.typeOneList[0].orgpoint.y<30)){
            elementflag = AcrossLeftRound;
            cout<<"turn into AcrossLeftRound!"<<endl;
        }
    }
    // if(lostrightside <= 4 && reallostleftside >= 15 && dofhead >= 75){
    //     if(LeftAngelList.List.size() >= 2){
    //         elementflag = AcrossLeftRound;
    //         cout<<"turn into AcrossLeftRound!"<<endl;
    //     }
    // }
}
/**************************************************************************************************************************/
//红色障碍物处理
Mat RedHinder;
int RedCount = 0;
bool lastRed = false;
bool Redflag = false;
int Redsencount = 5;
bool RecRed(Mat &src){
    Mat hsv,mask;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    Scalar RedLower(145, 43, 46);
    Scalar RedUpper(180, 255, 255);
    inRange(hsv, RedLower, RedUpper, mask);
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.size() == 0) {
         return false;
    }else {
        cv::Rect TempRect;
        for(uint8 i = 0;i < contours.size();i++){
            TempRect = boundingRect(contours[i]);
            if(TempRect.area() > 1100 && TempRect.width > TempRect.height){
                rectangle(src, TempRect, Scalar(0,255,0));
                return true;
                break;
            }
         }
         return false;
    }
}
/**************************************************************************************************************************/
void RecYellowCone(Mat src,double xishu,int yuzhi){
    Mat Cone_mat = cv::Mat(CamH,CamW,CV_8UC1);
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
   
    for(int i = 0; i < src.rows; ++i){
        cv::Vec3b *pixel = src.ptr<cv::Vec3b>(i); // point to first pixel in row
        for(int j = 0; j < src.cols; ++j){

        // 绿-蓝
            int diff1 = pixel[j][1] - (int)pixel[j][0] * xishu; //省赛1.1
            if (diff1 < 0) 
                diff1 = 0;
            else if(diff1 > yuzhi)
                diff1 = 254;
            Cone_mat.at<uchar>(i,j) = diff1;
        }
    }

  dilate(Cone_mat, Cone_mat, kernel_3, Point(-1, -1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());
 
  vector<vector<Point>> contours_cone;
  vector<Rect> rectlist_cone;//锥桶识别结果存储列表
  conelist.clear();
  findContours(Cone_mat, contours_cone, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  for (vector<Point> j : contours_cone) {
        Rect result = boundingRect(j);
        if(result.area() < 40)
            continue;
        else{
            TrDetectResult temp;
            temp.x = result.width/2 +result.x;
            temp.y = result.height/2 +result.y;
            temp.width = result.width;
            temp.height = result.height;
            temp.type = 3;
            conelist.push_back(temp);
        }
      //draw_point(m1,result.x + result.width/2,result.y + result.height/2,CV_COLOR_GOLD);
      //rectangle(m1,result,CV_COLOR_RED,1);
  }
}
/**************************************************************************************************************************/
//状态复位计数位
int32 OutCrossCount = 0;
int32 OutPreRightRoundCount = 0;
int32 OutOutRightRoundCount = 0;
int32 OutAcrossrightRoundCount = 0;
int32 OutPreLeftRoundCount = 0;
int32 OutOutLeftRoundCount = 0;
int32 OutAcrossleftRoundCount = 0;
int32 OutPreRound = 0;//入环岛鲁棒性保证
int32 OutRoundCount = 0;//出环岛鲁棒性保证
int32 RoundBaodiCount = 0;
int32 OutZebraCount = 0;
int SumZebraCount = 1;//一过斑马线而不入
bool ZebraEnable = true;
int32 CountZebraEnable = 0;
int Zebraspeedcount = 0;
/*******************************************维修区变量*******************************************/
bool MaintenWay = true;//维修区方向标志位true往左拐，false往右拐
int MaintenCountPatient = 0;
int MaintenCountTumble = 0;
int MaintenCountEvil = 0;
int MaintenCountThief = 0;
int CountHaveCone = 0;
int CountOutMainten = 0;
bool InMaintenFlag = false;
bool MaintenEnable = true;
/*******************************************炸弹区变量*******************************************/
bool BombEnable = true;
int BombCount = 0;
int BridgeCount = 0;
int OutBridgeCount = 0;
int YellowHinderCount = 0;
bool YellowHinderWay = true;//危险区寻边方向，true代表锥桶在左边寻右边线，false寻左边线
bool blockway = true;//黑色块方向，在左边为true，右边为false
bool HinderWayMetux = false;
bool EnableTurn = false;
int32 OutHinderCount = 180;
/*******************************************道具小车变量*******************************************/
int PropCountSafety = 0;
int PropCountDanger = 0;
int PropCountSpy = 0;
bool PropWay = false;//道具小车初始的位置，true为在右边，false为在左边
int PropIndex = 0;
int PropOutCount = 0;//道具小车状态专用退出标志
bool PrePropFlag = false;//预进入道具小车状态减速
//congfig控制变量-------------------------------------------------------------
bool CrossStateEnable = false;//十字小状态使能
int InMaintenCount = 4;//数几帧进维修区
int OutConeDIstance = 34;//进维修区时，锥桶离车头有多少距离时退出
int RightRoundTime = 2;//右环岛准入次数
int LeftRoundTime = 2; //左环岛准入次数
int PreRoundSpeed = 70;//预进环岛速度
int InRoundSpeed = 70;//进环岛速度
int OutRoundSpeed = 70;//出环岛速度
int InRightBoundPianyi = 29;//入右环岛偏移加强
int InLeftBoundPianyi = 28;//入左环岛偏移加强
int OutRightBoundPianyi = 29;//出右环岛偏移加强
int OutLeftBoundPianyi = 28;//出左环岛偏移加强
int PreMaintenSpeed = 50;//预进维修区速度
int InMaintenSpeed = 40;//进入维修区的速度
int LeftMaintenWay = -33;//进左维修区舵机打脚
int RightMaintenWay = 33;//进右维修区舵机打脚
int OutMaintenSpeed = -15;//出维修区往后倒车速度
int InBoomCount = 3;//数几帧进炸弹区
int PreBoomSpeed = 60;//预进炸弹区速度
int InBoomWay = 0;//手动控制进炸弹区第一个锥桶的位置，1为在右边，2为在左边，其他值不使能该功能
int YellowCount = 1;//经过了几个锥桶
int YellowHinderSpeed = 50;//在炸弹区域里的速度
int YellowChangeLine = 80;//在炸弹区，锥桶离车头距离，切换方向的y坐标
int LeftHinderPianyi = 23;//在炸弹区，巡左边线距离
int RightHinderPianyi = 23;//在炸弹区，巡右边线距离
int BlockPianyi = 9;//炸弹区域最后沿着黑块另外一边出来
int BlockPianLine = 65;//什么时候在砖头前偏头
int BridgeSpeed = 75;//上坡道的速度
int BridgeEnable = true;//桥标志位使能
int InPropCount = 5;//数几帧进追逐区
int PreSpySpeed = 75;//预逼停状态速度
int OverSpySpeed = 20;//掠过逼停状态速度
int StopSpySpeed = -1;//逼停状态速度
int StopSpyCount = 300;//逼停状态停止帧数
int OverSpyCount = 150;//掠过逼停状态帧数
int PreDangerSpeed = 40;//预碰撞状态速度
int SafetyRightPian = 22;//安全小车巡右边线偏移
int SafetyLeftPian = 22;//安全小车巡左边线偏移
int DangerRightPian = 10;//碰撞小车巡右边线偏移
int DangerLeftPian = 10;//碰撞小车巡左边线偏移
/**************************************************************************************************************************/
void ReconElements(void){

    for(int i = 0;i < 10;i++){
        detectTypeNum[i] = 0;
    }

    //conelist.clear();
    if(!Tr_result.empty()){
        int tempindex = 0;
        for(TrDetectResult i : Tr_result){

            detectTypeNum[i.type]++;
                    
            // if(i.type == 3)
            //      conelist.push_back(i);

            if(i.type == 2 || i.type == 4 || i.type == 7 )
                PropIndex = tempindex;

            tempindex++;
        }
    }

    if(elementflag != Normal)
        crossflag = None;

    switch(elementflag)
    {
        case Normal:

            if(CountZebraEnable < 300){
                CountZebraEnable++;
            }else if(CountZebraEnable >= 300){
                recozebra();
            }

            if(RightRoundTime > 0)
                RecPreRightRound();
            if(LeftRoundTime > 0)
                RecPreLeftRound();
//下标从0开始，type分别对应
//bomb bridge safety cone danger evil patient spy thief tumble
// 0     1      2      3    4     5      6     7    8    9 
            //*********************************识别维修区*********************************
            if(detectTypeNum[6] >= 1 && detectTypeNum[9] == 0 && detectTypeNum[5] == 0 && detectTypeNum[8] == 0){
                MaintenCountPatient++;
                MaintenCountTumble = 0;
                MaintenCountEvil = 0;
                MaintenCountThief = 0;
                if(MaintenCountPatient >= InMaintenCount &&MaintenEnable){
                    elementflag = RecMaintenWay;//识别到病人patient，左拐
                    MaintenWay = true;
                    MaintenCountPatient = 0;
                    cout<<"Left turn into RecMaintenWay!"<<endl;
                }
            }else if(detectTypeNum[6] == 0 && detectTypeNum[9] >= 1 && detectTypeNum[5] == 0 && detectTypeNum[8] == 0){
                MaintenCountTumble++;
                MaintenCountPatient = 0;
                MaintenCountEvil = 0;
                MaintenCountThief = 0;
                if(MaintenCountTumble >= InMaintenCount &&MaintenEnable){
                    elementflag = RecMaintenWay;//识别到摔倒tumble，左拐
                    MaintenWay = true;
                    MaintenCountTumble = 0;
                    cout<<"Left turn into RecMaintenWay!"<<endl;
                }
            }else if(detectTypeNum[6] == 0 && detectTypeNum[9] == 0 && detectTypeNum[5] >= 1 && detectTypeNum[8] == 0){
                MaintenCountEvil++;
                MaintenCountPatient = 0;
                MaintenCountTumble = 0;
                MaintenCountThief = 0;
                if(MaintenCountEvil >= InMaintenCount &&MaintenEnable){
                    elementflag = RecMaintenWay;//识别到歹徒evil，右拐
                    MaintenWay = false;
                    MaintenCountEvil = 0;
                    cout<<"Right turn into RecMaintenWay!"<<endl;
                }
            }else if(detectTypeNum[6] == 0 && detectTypeNum[9] == 0 && detectTypeNum[5] == 0 && detectTypeNum[8] >= 1){
                MaintenCountThief++;
                MaintenCountPatient = 0;
                MaintenCountTumble = 0;
                MaintenCountEvil = 0;
                if(MaintenCountThief >= InMaintenCount &&MaintenEnable){
                    elementflag = RecMaintenWay;//识别到小偷thief，右拐
                    MaintenWay = false;
                    MaintenCountThief = 0;
                    cout<<"Right turn into RecMaintenWay!"<<endl;
                }
            }else{
                MaintenCountPatient = 0;
                MaintenCountTumble = 0;
                MaintenCountEvil = 0;
                MaintenCountThief = 0;
            }
            //*********************************识别炸弹区*********************************
            if(detectTypeNum[0] >= 1 && BombEnable){
                
                if(BombCount >=InBoomCount){
                    elementflag = PreBoom;
                    cout<<"turn into PreBoom!"<<endl;
                    BombCount = 0;
                    BombEnable = false;
                }else if(BombCount >= 0 && BombCount < InBoomCount + 1)
                    BombCount++;
            }
            //*********************************识别过坡道*********************************
            if(detectTypeNum[1] >= 1 && dofhead > 72 && BridgeEnable){
                
                if(BridgeCount >=5){
                    elementflag = Bridge;
                    cout<<"turn into Bridge!"<<endl;
                    BridgeCount = 0;
                    BridgeEnable = false;
                }else if(BridgeCount >= 0 && BridgeCount < 5)
                    BridgeCount++;
            }else{
                BridgeCount = 0;
            }


            //*********************************识别道具小车*********************************

            if(Topline < 10 && reallostleftside < 2 && reallostrightside < 2 && lostleftside < 2 && lostrightside < 2){
                PrePropFlag = false;
                //用传统视觉预识别到道具小车之类的砖块，要预降速
                if((RightAngelList.List.empty() || RightAngelList.List[0].num > 75)&& LeftAngelList.List.size() >= 3&& LeftAngelList.typeThreeList.size()>=2){

                    if(LeftAngelList.typeThreeList[0].orgpoint.y - LeftAngelList.typeThreeList[1].orgpoint.y <= 25
                        && (LeftAngelList.List[1].angletype == 1 || LeftAngelList.List[1].angletype == 2)
                    && LeftAngelList.List[0].angletype == -1 && (LeftAngelList.List[2].angletype == -1 || LeftAngelList.List[3].angletype == -1)){
                        PrePropFlag = true;
                        printf("pre left prop speed slow\n");
                    }
                }else if((LeftAngelList.List.empty() || LeftAngelList.List[0].num > 75)&& RightAngelList.List.size() >= 3&& RightAngelList.typeThreeList.size()>=2){
                    if(RightAngelList.typeThreeList[0].orgpoint.y - RightAngelList.typeThreeList[1].orgpoint.y <= 25
                        && (RightAngelList.List[1].angletype == 1 || RightAngelList.List[1].angletype == 2)
                    && RightAngelList.List[0].angletype == -1 && (RightAngelList.List[2].angletype == -1 || RightAngelList.List[3].angletype == -1)){
                        PrePropFlag = true;
                        printf("pre Right prop speed slow\n");
                    }
                }
            }

            if(detectTypeNum[2] >= 1 && detectTypeNum[4] == 0 && detectTypeNum[7] == 0){
                PropCountSafety++;
                PropCountDanger = 0;
                PropCountSpy = 0;
                if(PropCountSafety >= InPropCount &&(LeftAngelList.List.size() >= 3 || RightAngelList.List.size() >= 3)){

                    elementflag = GoSafety;//识别safety,直接掠过
                    PropCountSafety = 0;
                    
                    if(Tr_result[PropIndex].x < CamW/2 && LeftAngelList.List.size() >= 3){//道具小车在左边
                        PropWay = false;
                        cout<<"Left turn into GoSafety!"<<endl;
                    }else if(Tr_result[PropIndex].x > CamW/2 && RightAngelList.List.size() >= 3){//道具小车在右边
                        PropWay = true;
                        cout<<"Right turn into GoSafety!"<<endl;
                    }
                }
            }else if(detectTypeNum[2] == 0 && detectTypeNum[4] >= 1 && detectTypeNum[7] == 0){
                PropCountSafety = 0;
                PropCountDanger++;
                PropCountSpy = 0;
                if(PropCountDanger >= InPropCount){
                    elementflag = PreDanger;//识别到danger,准备撞击
                    PropCountDanger = 0;
                    
                    if(Tr_result[PropIndex].x < CamW/2){//道具小车在左边
                        PropWay = false;
                        cout<<"Left turn into PreDanger!PropWay:"<<PropWay<<endl;
                    }else if(Tr_result[PropIndex].x > CamW/2){//道具小车在右边
                        PropWay = true;
                        cout<<"Right turn into PreDanger!PropWay:"<<PropWay<<endl;
                    }
                }
            }else if(detectTypeNum[2] == 0 && detectTypeNum[4] == 0 && detectTypeNum[7] >= 1){
                PropCountSafety = 0;
                PropCountDanger = 0;
                PropCountSpy++;
                if(PropCountSpy >= InPropCount){
                             
                    if(Tr_result[PropIndex].x < CamW/2){//道具小车在左边
                        PropWay = false;
                        cout<<"Left turn into PreSpy!PropWay:"<<PropWay<<endl;
                        elementflag = PreSpy;//识别到PreSpy,准备逼停
                        PropCountSpy = 0;
                    }else if(Tr_result[PropIndex].x > CamW/2){//道具小车在右边
                        PropWay = true;
                        cout<<"Right turn into PreSpy!PropWay:"<<PropWay<<endl;
                        elementflag = PreSpy;//识别到PreSpy,准备逼停
                        PropCountSpy = 0;
                    }
                }
            }else{
                PropCountSafety = 0;
                PropCountDanger = 0;
                PropCountSpy = 0;
            }


            MakePreCrossLine();

            waycontrolline1 = CONTROLLINEONE;//第一打角行
            waycontrolline2 = CONTROLLINETWO;//第二打角行

            controsideflag = MidSide;
            

            if(CrossStateEnable){

                switch(crossflag){
                    case None:

                    if(CrossSumCount < 20){
                        CrossSumCount++;
                        if(CrossInCount >= 10){
                            CrossInCount = 0;
                            CrossSumCount = 0;
                            crossflag = HaveCrossed;
                            printf("small state turn into HaveCrossed\n");
                        }
                    }else if(CrossSumCount >= 20){
                        CrossSumCount = 0;
                        CrossInCount = 0;
                    }

                    break;
                    case HaveCrossed:
                        
                        if(lostleftside < 1 &&leftside[length_leftside - 1].x >= CamW * 3 / 5 &&lostright_pos_end > 32
                            &&reallostrightside > 18 &&reallostleftside < 2 && length_rightside < 85){
                            InAlphaCount++;
                            if(InAlphaCount >= 20){
                                InAlphaCount = 0;
                                crossflag = InRightAlpha;
                                printf("small state turn into InRightAlpha\n");
                            }
                        }else if(lostrightside < 1 &&rightside[length_rightside - 1].x <= CamW * 2 / 5 &&lostleft_pos_end > 30
                            &&reallostleftside > 14 &&reallostrightside < 2 && length_leftside < 85){
                            InAlphaCount++;
                            if(InAlphaCount >= 20){
                                InAlphaCount = 0;
                                crossflag = InLeftAlpha;
                                printf("small state turn into InLeftAlpha\n");
                            }
                        }else{
                            InAlphaCount = 0;
                        }

                        if(CrossOutCount <= 300){//鲁棒性处理
                            CrossOutCount++;
                            if(CrossOutCount == 300){
                                CrossOutCount = 0;
                                crossflag = None;
                                printf("small state turn into None from HaveCrossed timeout!\n");
                            }
                        }
                    break;

                    case InRightAlpha:

                        if(startlostrightside <= waycontrolline1 && reallostrightside >= 40){
                            controsideflag = RightSide;
                            printf("InRightAlpha out buxian!\n");
                            Sidepianyi = 35;
                        }

                        if(crosslostrightside <= 2||!RightAngelList.typeOneList.empty()){
                            crossflag = None;
                            printf("small state turn into None from InRightAlpha active!\n");
                        }

                        if(CrossOutCount <= 360){//鲁棒性处理
                            CrossOutCount++;
                            if(CrossOutCount == 360){
                                CrossOutCount = 0;
                                crossflag = None;
                                printf("small state turn into None from InRightAlpha timeout!\n");
                            }
                        }

                    break;
                    case InLeftAlpha:

                        if(startlostleftside <= waycontrolline1 && reallostleftside >= 40){
                            controsideflag = LeftSide;
                            printf("InLeftAlpha out buxian!\n");
                            Sidepianyi = 35;
                        }

                        if(crosslostleftside <= 2||!LeftAngelList.typeOneList.empty()){
                            crossflag = None;
                            printf("small state turn into None from InLeftAlpha active!\n");
                        }

                        if(CrossOutCount <= 360){//鲁棒性处理
                            CrossOutCount++;
                            if(CrossOutCount == 360){
                                CrossOutCount = 0;
                                crossflag = None;
                                printf("small state turn into None from InLeftAlpha timeout!\n");
                            }
                        }

                    break;
                }
            }



        break;
        case PreRightRound:
            
            Sidepianyi = 36;
            controsideflag = LeftSide;
            RecInRightRound();
            if(OutPreRightRoundCount < 2)
            OutPreRightRoundCount++;
            else if(OutPreRightRoundFlag && OutPreRightRoundCount >= 2){
            OutPreRightRoundCount = 0;
            OutPreRightRoundFlag = false;
            cout<<"turn into InRightRound!"<<endl;
            elementflag = InRightRound;
            RoundBaodiCount = 120;
            }
            //以下为鲁棒性部分
            if(OutPreRound < 350){
                OutPreRound++;
            }else if(OutPreRound == 350){
                OutPreRound = 0;
                elementflag = Normal;
            }
        break;
        case InRightRound:
            OutPreRound = 0;
            
            
            if(OutOutRightRoundCount < 80){//原为120帧************************************************************
                OutOutRightRoundCount++;
                controsideflag = RightSide;
                Sidepianyi = InRightBoundPianyi;//
                waycontrolline1 = 58;//第一打角行
                waycontrolline2 = 59;//第二打角行
                if(OutOutRightRoundCount > 50){
                    if(lostleftside < 1 &&leftside[length_leftside - 1].x >= CamW * 3 / 5 ){
                        controsideflag = MidSide;
                        waycontrolline1 = CONTROLLINEONE;//第一打角行
                        waycontrolline2 = CONTROLLINETWO;//第二打角行
                        printf("Mid in Inright\n");
                    }
                }
            }else{
                if(lostleftside < 1 &&leftside[length_leftside - 1].x >= CamW * 3 / 5){
                    controsideflag = MidSide;
                    waycontrolline1 = CONTROLLINEONE;//第一打角行
                    waycontrolline2 = CONTROLLINETWO;//第二打角行
                    printf("Mid in Inright\n");
                }
                RecOutRightRound();
            }
            
           
            
        break;
        case OutRightRound:
            Sidepianyi = OutRightBoundPianyi;//
            OutOutRightRoundCount = 0;
            controsideflag = RightSide;
            waycontrolline1 = 55;//第一打角行
            waycontrolline2 = 56;//第二打角行
            //RoadK(1,90,50);
            RecAcrossrightRound();
            if(OutRoundCount < 300){
                OutRoundCount++;
            }else if(OutRoundCount == 300){
                OutRoundCount = 0;
                elementflag = Normal;
                RightRoundTime--;
            }
        break;
        case AcrossRightRound:
            OutRoundCount = 0;
            Sidepianyi = 30;
            waycontrolline1 = 55;//第一打角行
            waycontrolline2 = 56;//第二打角行
            controsideflag = LeftSide;//
            //退出标志位计时
            if(OutAcrossrightRoundCount < 50)
                OutAcrossrightRoundCount++;
            else{
                OutAcrossrightRoundCount = 0;
                elementflag = Normal;
                RightRoundTime--;
            }
           
        break;
        case PreLeftRound:
            Sidepianyi = 36;
            controsideflag = RightSide;
            RecInLeftRound();
            if(OutPreLeftRoundCount < 2)
            OutPreLeftRoundCount++;
            else if(OutPreLeftRoundFlag && OutPreLeftRoundCount >= 2){
            OutPreLeftRoundCount = 0;
            OutPreLeftRoundFlag = false;
            cout<<"turn into InLeftRound!"<<endl;
            elementflag = InLeftRound;
            RoundBaodiCount = 120;
            }
            //以下为鲁棒性部分
            if(OutPreRound < 350){
                OutPreRound++;
            }else if(OutPreRound == 350){
                OutPreRound = 0;
                elementflag = Normal;
            }
        break;
        case InLeftRound:
            OutPreRound = 0;
            
            
            if(OutOutLeftRoundCount < 80){//原为120帧//
                OutOutLeftRoundCount++;
                controsideflag = LeftSide;
                Sidepianyi = InLeftBoundPianyi;//
                waycontrolline1 = 58;//第一打角行
                waycontrolline2 = 59;//第二打角行
                if(OutOutLeftRoundCount > 50){
                    if(lostrightside < 1 &&rightside[length_rightside - 1].x <= CamW * 2 / 5 ){
                        controsideflag = MidSide;
                        waycontrolline1 = CONTROLLINEONE;//第一打角行
                        waycontrolline2 = CONTROLLINETWO;//第二打角行
                        printf("Mid in Inleft\n");
                    }
                }
            }else{
                if(lostrightside < 1 &&rightside[length_rightside - 1].x <= CamW * 2 / 5 ){
                    controsideflag = MidSide;
                    waycontrolline1 = CONTROLLINEONE;//第一打角行
                    waycontrolline2 = CONTROLLINETWO;//第二打角行
                    printf("Mid in Inleft\n");
                }
                RecOutLeftRound();
            }
           
           
        break;
        case OutLeftRound:
            Sidepianyi = OutLeftBoundPianyi;//
            controsideflag = LeftSide;
            waycontrolline1 = 55;//第一打角行
            waycontrolline2 = 56;//第二打角行
            //RoadK(0,90,45);
            RecAcrossLeftRound();
            if(OutRoundCount < 200){
                OutRoundCount++;
            }else if(OutRoundCount == 200){
                OutRoundCount = 0;
                elementflag = Normal;
                LeftRoundTime--;
            }
        break;
        case AcrossLeftRound:
            OutRoundCount = 0;
            Sidepianyi = 30;
            waycontrolline1 = 55;//第一打角行
            waycontrolline2 = 56;//第二打角行
            controsideflag = RightSide;//
            //退出标志位计时
            if(OutAcrossleftRoundCount < 50)
            OutAcrossleftRoundCount++;
            else{
            OutAcrossleftRoundCount = 0;
            elementflag = Normal;
            LeftRoundTime--;
            }
            
        break;
        case ZebraFlag:
            //退出标志位计时
           
            controsideflag = MidSide;
            //controsideflag = Straight;

            // if(OutZebraCount < 20)
            // OutZebraCount++;
            // else{
            // OutZebraCount = 0;
            // SumZebraCount++;
            
            // if(SumZebraCount != 3)
            // elementflag = Normal;
            // }
            // CountZebraEnable = 0;

        break;
        case RecMaintenWay:

            if(conelist.size() >=3)
                CountHaveCone++;
            else
                CountHaveCone = 0;
            
            if(CountHaveCone >= 2){
                CountHaveCone = 0;
                cout<<"turn into preMainten!"<<endl;
                elementflag = PreMainten;
            }
            
        break;
        case PreMainten:
            if(conelist.empty())
                break;

            for (int i = 0; i < conelist.size();i++) {//锥桶按照y坐标排序,y大的在前面
                for (int j = i + 1; j < conelist.size(); j++) {
                    if (conelist[i].y < conelist[j].y) {
                        TrDetectResult temp = conelist[i];
                        conelist[i] = conelist[j];
                        conelist[j] = temp;
                    }
                }
            }

            if(conelist[0].y > 60)
                InMaintenFlag = true;
                
            if(InMaintenFlag &&conelist[0].y < 50 &&conelist.back().y > 3){
                InMaintenFlag = false;
                elementflag = InMainten;
                cout<<"turn into InMainten"<<endl;
            }
        
        break;
        case InMainten:

            for (int i = 0; i < conelist.size();i++) {//锥桶按照y坐标排序,y大的在前面
                for (int j = i + 1; j < conelist.size(); j++) {
                    if (conelist[i].y < conelist[j].y) {
                        TrDetectResult temp = conelist[i];
                        conelist[i] = conelist[j];
                        conelist[j] = temp;
                    }
                }
            }

            for(TrDetectResult i : conelist){
                if(i.y > OutConeDIstance && i.x > 55 && i.x < 105){
                    elementflag = OutMainten;
                }
            }
        
        break;
        case OutMainten:

            if(dofhead > 60){
                elementflag = Normal;
                MaintenEnable = false;
                cout<<"turn into normal"<<endl;
            }
            CountOutMainten = 90;
        
        break;
        case PreBoom:

            if(!conelist.empty()){
                YellowHinderCount++;
                if(YellowHinderCount >=4){
                    if(conelist.size() > 1){
                        for (int i = 0; i < conelist.size();i++) {//锥桶按照y坐标排序,y大的在前面
                            for (int j = i + 1; j < conelist.size(); j++) {
                                if (conelist[i].y < conelist[j].y) {
                                    TrDetectResult temp = conelist[i];
                                    conelist[i] = conelist[j];
                                    conelist[j] = temp;
                                }
                            }
                        }
                    }

                    if(conelist[0].x <= Mid)//原
                        YellowHinderWay = true;
                    else if(conelist[0].x > Mid)
                        YellowHinderWay = false;


                    if(conelist[0].y > 45){

                        if(InBoomWay == 1){
                            YellowHinderWay = false;
                            printf("control in hand first right\n");
                        }else if(InBoomWay == 2){
                            YellowHinderWay = true;
                            printf("control in hand first left\n");
                        }else{
                            printf("auto juge way\n");
                        }

                        elementflag = YellowHinder;
                        YellowHinderCount = 0;
                        cout<<"turn into YellowHinder:"<<YellowHinderWay<<endl;
                    }
                }
            }
        
        break;
        case YellowHinder:


            if(conelist.size() > 1){
                for (int i = 0; i < conelist.size();i++) {//锥桶按照y坐标排序,y大的在前面
                    for (int j = i + 1; j < conelist.size(); j++) {
                        if (conelist[i].y < conelist[j].y) {
                        TrDetectResult temp = conelist[i];
                        conelist[i] = conelist[j];
                        conelist[j] = temp;
                        }
                    }
                }
            }

            if(!conelist.empty()){

                if(HinderWayMetux && (conelist[0].y - (conelist[0].height/2))  > YellowChangeLine && YellowCount > 0){
                    EnableTurn  = true;
                    // HinderWayMetux = false;
                    // if(YellowHinderWay ){
                    //     YellowHinderWay = false;
                    // }else{
                    //     YellowHinderWay = true;
                    // }
                    // YellowCount--;
                    // cout<<"Have changed the way:"<<YellowHinderWay<<endl;
                }
                if(EnableTurn && conelist[0].y < YellowChangeLine && conelist[0].y >= 48){
                    HinderWayMetux = false;
                    EnableTurn  = false;
                    if(YellowHinderWay ){
                        YellowHinderWay = false;
                    }else{
                        YellowHinderWay = true;
                    }
                    YellowCount--;
                    cout<<"Have changed the way:"<<YellowHinderWay<<endl;

                }

            
                if((conelist[0].y - (conelist[0].height/2)) <= 50){
                    HinderWayMetux = true;
                }
                
            }

            if(!LeftAngelList.typeTwoList.empty()&&!LeftAngelList.typeThreeList.empty() &&conelist.empty()){
                if(LeftAngelList.typeTwoList[0].orgpoint.y > BlockPianLine &&LeftAngelList.typeTwoList[0].orgpoint.y > BlockPianLine){
                    
                    blockway = true;
                    cout<<"turn into OutHinder in left block"<<endl;

                    controsideflag = RightSide;
                    Sidepianyi = BlockPianyi;
                    elementflag = OutHinder;
                    break;
                }
            }

            if(!RightAngelList.typeTwoList.empty()&&!RightAngelList.typeThreeList.empty() &&conelist.empty()){
                if(RightAngelList.typeTwoList[0].orgpoint.y > BlockPianLine &&RightAngelList.typeTwoList[0].orgpoint.y > BlockPianLine){
                    
                    blockway = false;
                    cout<<"turn into OutHinder in right block"<<endl;

                    controsideflag = LeftSide;
                    Sidepianyi = BlockPianyi;
                    elementflag = OutHinder;
                    break;
                }
            }


            if(YellowHinderWay){
                Sidepianyi = RightHinderPianyi;
                controsideflag = RightSide;
            }else{
                Sidepianyi = LeftHinderPianyi;
                controsideflag = LeftSide;
            }

            waycontrolline1 = 59;//第一打角行
            waycontrolline2 = 60;//第二打角行
            
        
        break;
        case OutHinder:
            if(blockway && LeftAngelList.typeOneList.empty() && LeftAngelList.typeThreeList.empty()){
                
                elementflag = Normal;
                cout<<"turn into Normal from OutHinder"<<endl;
            }else if(blockway && !LeftAngelList.typeOneList.empty() 
                    &&(LeftAngelList.typeOneList[0].orgpoint.y > 73 || LeftAngelList.typeOneList[0].orgpoint.y < 20)){
                
                elementflag = Normal;
                cout<<"turn into Normal from OutHinder"<<endl;
            }

            if(!blockway && RightAngelList.typeOneList.empty() && RightAngelList.typeThreeList.empty()){
                
                elementflag = Normal;
                cout<<"turn into Normal from OutHinder"<<endl;
            }else if(!blockway && !RightAngelList.typeOneList.empty()
                    &&(RightAngelList.typeOneList[0].orgpoint.y > 73 || RightAngelList.typeOneList[0].orgpoint.y < 20)){
                
                elementflag = Normal;
                cout<<"turn into Normal from OutHinder"<<endl;
            }

            if(OutHinderCount > 0){
                OutHinderCount--;
            }else if(OutHinderCount == 0){
                elementflag = Normal;
                cout<<"turn into Normal from OutHinder:time out!"<<endl;
            }

            if(blockway){
                controsideflag = RightSide;
                Sidepianyi = BlockPianyi;
            }else{
                controsideflag = LeftSide;
                Sidepianyi = BlockPianyi;
            }

            waycontrolline1 = 59;//第一打角行
            waycontrolline2 = 60;//第二打角行
            

        break;
        case GoSafety:
            
            if(PropWay){//道具小车在右边，应该巡线左边
                Sidepianyi = SafetyLeftPian;
                controsideflag = LeftSide;
            }else{//道具小车在左边，应该巡线右边
                Sidepianyi = SafetyRightPian;
                controsideflag = RightSide;
            }

            PropOutCount++;
            if(PropOutCount >= 90 && detectTypeNum[2] == 0){
                if(PropWay && (RightAngelList.List.empty() || RightAngelList.List[0].orgpoint.y <= 40)){
                    PropOutCount = 0;
                    cout<<"turninto normal from safety!"<<endl;
                    elementflag = Normal;
                }else if(!PropWay && (LeftAngelList.List.empty() || LeftAngelList.List[0].orgpoint.y <= 40)){
                    PropOutCount = 0;
                    cout<<"turninto normal from safety!"<<endl;
                    elementflag = Normal;
                }else if(PropOutCount >= 180){
                    PropOutCount = 0;
                    cout<<"turninto normal from safety!:time out!"<<endl;
                    elementflag = Normal;
                }
                
            }

            waycontrolline1 = 59;//第一打角行
            waycontrolline2 = 60;//第二打角行

        break;
        case PreDanger:

            if(PropWay){//道具小车在右边，贴着右边巡线
                Sidepianyi = DangerRightPian;
                controsideflag = RightSide;
            }else{//道具小车在左边，贴着左边巡线
                Sidepianyi = DangerLeftPian;
                controsideflag = LeftSide;
            }

            PropOutCount++;
            if(PropOutCount >= 60 && detectTypeNum[4] == 0){//鲁棒性保险
                if(PropWay && (RightAngelList.List.empty() || RightAngelList.List[0].orgpoint.y <= 45)){
                    PropOutCount = 0;
                    cout<<"turninto normal from PreDanger!"<<endl;
                    elementflag = Normal;
                }else if(!PropWay && (LeftAngelList.List.empty() || LeftAngelList.List[0].orgpoint.y <= 45)){
                    PropOutCount = 0;
                    cout<<"turninto normal from PreDanger!"<<endl;
                    elementflag = Normal;
                }else if(PropOutCount >= 180){
                    PropOutCount = 0;
                    cout<<"turninto normal from PreDanger!:time out!"<<endl;
                    elementflag = Normal;
                }
            }
     
        break;
        case PreSpy:

            if(PropWay){//道具小车在右边，应该巡线左边
                Sidepianyi = 22;
                controsideflag = LeftSide;
            }else{//道具小车在左边，应该巡线右边
                Sidepianyi = 26;
                controsideflag = RightSide;
            }

            PropOutCount++;
            if(PropOutCount >= 5){
                if(PropWay && (RightAngelList.List.empty() || RightAngelList.List[0].orgpoint.y <= 30) 
                    && length_half_rightside <= 67){
                    PropOutCount = 0;
                    elementflag = OverSpy;
                    cout<<"turn into OverSpy!PropWay:"<<PropWay<<endl;
                }else if(!PropWay && (LeftAngelList.List.empty() || LeftAngelList.List[0].orgpoint.y <= 30) 
                    && length_half_leftside <= 67){
                    PropOutCount = 0;
                    elementflag = OverSpy;
                    cout<<"turn into OverSpy!PropWay:"<<PropWay<<endl;
                }else if(PropOutCount >= 360){
                    PropOutCount = 0;
                    elementflag = OverSpy;
                    cout<<"turn into OverSpy!:time out!"<<endl;
                }
                
            }

            waycontrolline1 = 59;//第一打角行
            waycontrolline2 = 60;//第二打角行

        break;
        case OverSpy:

            if(PropWay){//道具小车初始在右边，贴着右边等他
                Sidepianyi = 14;
                controsideflag = RightSide;
            }else{//道具小车初始在左边，贴着左边等他
                Sidepianyi = 14;
                controsideflag = LeftSide;
            }

            if(PropWay && reallostrightside < 2 &&dofhead <= 40){
                PropOutCount = 0;
                cout<<"turn into StopSpy!"<<endl;
                elementflag = StopSpy;
            }else if(!PropWay && reallostleftside < 2 &&dofhead <= 40){
                PropOutCount = 0;
                cout<<"turn into StopSpy!"<<endl;
                elementflag = StopSpy;
            }

            PropOutCount++;
            if(PropOutCount >= OverSpyCount){
                PropOutCount = 0;
                cout<<"turn into StopSpy:time out!"<<endl;
                elementflag = StopSpy;
            }
            waycontrolline1 = 59;//第一打角行
            waycontrolline2 = 60;//第二打角行
     
        break;
        case StopSpy:
            PropOutCount++;
            if(PropOutCount >= StopSpyCount){
                PropOutCount = 0;
                elementflag = Normal;
                cout<<"turn into Normal from spy!"<<endl;
            }
     
        break;
        case Bridge:
            OutBridgeCount++;
            if(OutBridgeCount >= 180){
                OutBridgeCount = 0;
                elementflag = Normal;
                printf("turn into Normal from Bridge!\n");
            }

        break;
        default:

        break;
        
    }
}


int Sidepianyifitting(void){
    int result = 0;
    if(controsideflag == MidSide)
        return result;
    else if(controsideflag == RightSide){
        result = (CamW/2 - Sidepianyi - (CamW - midright[waycontrolline2]))*0.1;//**************************0.4
    }else if(controsideflag == LeftSide){
        result = (CamW/2 - Sidepianyi - midleft[waycontrolline2])*0.1;
    }
    if(result < 0)
    result = 0;
    else if(result > Sidepianyi)
    result = Sidepianyi;

    return result;
}
/*
int Sidepianyifitting(void){
    int result = 0;
    if(controsideflag == MidSide)
        return result;
    else if(controsideflag == RightSide){
        result = (CamW/2 - Sidepianyi - (CamW - midright[waycontrolline2]))*2/3;
    }else if(controsideflag == LeftSide){
        result = (CamW/2 - Sidepianyi - midleft[waycontrolline2])*5/6;
    }
    if(result < 0)
    result = 0;
    else if(result > Sidepianyi)
    result = Sidepianyi;

    return result;
}
*/

/*
函数功能：获取最终的的实时中线
参数说明：void
返回说明：void
备注：
*/
void getmidline(void){
    if(controsideflag == MidSide){                                                         //得到中线
        for (int i = CamH - 2; i > Topline; i--) 
            midline[i] = (midleft[i] + midright[i])/2;
    }else if(controsideflag == RightSide){
        for (int i = CamH - 2; i > Topline; i--) 
            midline[i] = midright[i] - (Sidepianyi - Sidepianyifitting());
    }else if(controsideflag == LeftSide){
        for (int i = CamH - 2; i > Topline; i--) 
            midline[i] = midleft[i] + (Sidepianyi - Sidepianyifitting());
    }else if(controsideflag == Straight){
         midline[waycontrolline1] = Mid; 
         midline[waycontrolline2] = Mid;
    }
}
int SpeedCount = 0;
void getcontroldata(uint8 *sendlist,int &control,int &motor,int SH,int SHR,int SM,int SMR,int SL){

  double datak = 1;

  if(waycontrolline1 <= Topline){
    realcontrolline1 = Topline + 1;
    realcontrolline2 = realcontrolline1 + 1;

    datak = (((double)Topline - (double)waycontrolline1)/10.0)*1.5 + datak;//Topline - waycontrolline1 大约在0到40之间
  }else{
    realcontrolline1 = waycontrolline1;
    realcontrolline2 = waycontrolline2;

  }

  if(realcontrolline2 > CamH - 15){
    realcontrolline1 = CamH - 15;
    realcontrolline2 = CamH - 15;
  }

  double cdata1 = (double)((int)midline[realcontrolline1 ] - Mid);
  double cdata2 = (double)((int)midline[realcontrolline2 ] - Mid);
//   cout<<"cdata1 = "<<cdata1<<endl;
//   cout<<"cdata2 = "<<cdata2<<endl;
  control = (int)((cdata1 * 0.7 + cdata2 * 0.3) * datak);
    
//   control = (int)(((double)((int)midline[realcontrolline1 ] - Mid)*0.6 + 
//                 (double)((int)midline[realcontrolline2 ] - Mid)*0.4)*datak);

  switch(elementflag)
    {
        case ZebraFlag://******************************斑马线
            
            if(Zebraspeedcount < 12){
                Zebraspeedcount++;
                sendlist[2] = 1;
                motor = 40;
            }else{
                sendlist[2] = 1;
                motor = -1;
            }
            
        break;
        case PreRightRound://******************************预识别右环岛
            sendlist[0] = 1;//
            sendlist[6] = 1;
       
            motor = PreRoundSpeed;//2024.5.27胡佑鑫 原数值65
        break;
        case InRightRound://******************************进右环岛
            sendlist[6] = 1;
            if(RoundBaodiCount > 0){
                RoundBaodiCount--;
                if(control < 26){
                    control = 26;
                }
            }
            motor = InRoundSpeed;//2024.5.27胡佑鑫 原数值72
        break;
        case OutRightRound://******************************出右环岛
            sendlist[6] = 1;
            motor = OutRoundSpeed;//2024.5.27胡佑鑫 原数值60
        break;
        case AcrossRightRound://******************************出左环岛
            sendlist[6] = 1;
            motor = OutRoundSpeed;//2024.5.27胡佑鑫 原数值55
        break;
        case PreLeftRound://******************************预识别左环岛
            sendlist[1] = 1;
            sendlist[6] = 1;
            motor = PreRoundSpeed;//2024.5.27胡佑鑫 原数值65
        break;
        case InLeftRound://******************************进左环岛
            // if(LeftRoundbaodiEnable){
            //     LeftRoundbaodicount--;
            //     if(LeftRoundbaodicount > 0){
            //         if(control > LeftRpundbaodidata){
            //             control = LeftRpundbaodidata;
            //         }
            //     }
            // }
            if(RoundBaodiCount > 0){
                RoundBaodiCount--;
                if(control > -26){
                    control = -26;
                }
            }
            sendlist[6] = 1;
            motor = InRoundSpeed;//2024.5.27胡佑鑫 原数值72
        break;
        case OutLeftRound://******************************出左环岛
            sendlist[6] = 1;
            motor = OutRoundSpeed;//2024.5.27胡佑鑫 原数值55
        break;
        case AcrossLeftRound://******************************出左环岛
            sendlist[6] = 1;
            motor = OutRoundSpeed;//2024.5.27胡佑鑫 原数值55
        break;
        case PreMainten://******************************预识别维修区
            sendlist[2] = 1;
            motor = PreMaintenSpeed;
        break;
        case InMainten://******************************进维修区
            sendlist[2] = 1;
            if(MaintenWay)
                control = LeftMaintenWay;
            else
                control = RightMaintenWay;

            motor = InMaintenSpeed;
        break;
        case OutMainten://******************************出维修区
            sendlist[2] = 1;
            if(MaintenWay)
                control = LeftMaintenWay;
            else
                control = RightMaintenWay;

            motor = OutMaintenSpeed;
        break;
        case PreBoom://******************************预识别炸弹区
            sendlist[2] = 1;
            motor = PreBoomSpeed;

        break;
        case YellowHinder://******************************在炸弹区里面
            sendlist[4] = 1;
            motor = YellowHinderSpeed;

        break;
        case OutHinder:
            sendlist[4] = 1;
            motor = YellowHinderSpeed + 5;
        break;
        case GoSafety:
            sendlist[2] = 1;

            motor = 85;

        break;
        case PreDanger:
            sendlist[2] = 1;

            motor = PreDangerSpeed;

        break;
        case PreSpy:
            sendlist[4] = 1;
            motor = PreSpySpeed;
            // if(PropWay){//道具小车初始在右边，贴着左边过
            //     if(control > -20){
            //         control = -20;
            //     }
            // }
            // else{//道具小车初始在左边，贴着右边过
            //     if(control < 20){
            //         control = 20;
            //     }
            // }

        break;
        case OverSpy:
            sendlist[4] = 1;
            motor = OverSpySpeed;
            // if(PropWay){//道具小车初始在右边，贴着右边等他
            //     if(control < 45){
            //         control = 45;
            //     }
            // }else{//道具小车初始在左边，贴着左边等他
            //     // if(control > -45){
            //     //     control = -45;
            //     // }
            // }

        break;
        case StopSpy:
            sendlist[4] = 1;
            motor = StopSpySpeed;

        break;
        case Bridge://******************************上坡道
            sendlist[2] = 1;
            motor = BridgeSpeed;

        break;
        default://******************************默认巡线状态

            if(CountOutMainten > 0){
                CountOutMainten--;
                sendlist[2] = 1;
                motor = 45;
                break;
            }

            if(dofhead >= 75){
                SpeedCount++;
                if(SpeedCount > 15){
                    SpeedCount = 15;
                }
            }else
                SpeedCount = 0;

            if(dofhead >= 75 && motor != 0){//当车头距离边界的距离在80到90之间
                if(dofhead > 90){
                    dofhead = 90;
                }
                if(SpeedCount >= 15)
                    motor = SH;
                else
                    motor = SM + SMR;
            }else if(dofhead < 75 && dofhead >= 60)
                motor = SM + SMR * (dofhead - 60) / 15;
            else if(dofhead < 60)
                motor = SL;

            if(PrePropFlag){
                sendlist[2] = 1;
                motor = 85;
            }
            
        break;

    }
}









