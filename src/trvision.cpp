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
roadside leftside[230] = { {0},{0} };//左边线八临域结构体，每次初始化
uint8 length_leftside = 0;//左边八临域结构体数组长度,每次初始化
roadside rightside[230] = { {0},{0} };//右边线八临域结构体，每次初始化
uint8 length_rightside = 0;//右边八临域结构体数组长度,每次初始化
uint8 midline[CamH] = { 0 };//拟合出来实际道路中线，每次初始化
uint8 midleft[CamH] = { 0 };//无重叠左边边线，每次初始化
uint8 validmidleft[CamH] = { 0 };//为了得到无重叠左边边线设置的验证数组,每次初始化
uint8 midright[CamH] = { 0 };//无重叠右边边线，每次初始化
uint8 validmidright[CamH] = { 0 };//为了得到无重叠右边边线设置的验证数组,每次初始化
uint8 reallostleftside = 0;//左边真实丢线数，每次初始化
uint8 startlostleftside = 0;//左边开始计逻辑丢线的位置，对应y坐标
uint8 lostleftside = 0;//左边丢线数，每次初始化
uint8 lostleft_pos_end = 0;//左边丢线结束寻找位置,对应y坐标
uint8 reallostrightside = 0;//右边真实丢线数，每次初始化
uint8 startlostrightside = 0;//右边开始计逻辑丢线的位置，对应y坐标
uint8 lostrightside = 0;//右边丢线数，每次初始化
uint8 lostright_pos_end = 0;//右边丢线结束寻找位置,对应y坐标
struct roadside cirpos[9] = { {0,0},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} };//八邻域寻找下一点时坐标变化查找表
                                                                                            //从12点方向逆时针旋转查找表
//逆透视变换矩阵
double change_un_Mat[3][3] ={{0.815152,-0.428477,16.334382},{-0.000000,0.200802,5.914327},{0.000000,-0.005972,0.824109}};                

int detectTypeNum[13] = {0}; 
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
     length_rightside = 0;
     memset(validmidleft, '\0', CamH);
     memset(validmidright, '\0', CamH);
     for(int i = 0;i < CamH;i++){
          midleft[i] = Mid;//0
          midright[i] = Mid;//CamW - 1
          midline[i] = Mid;
     }
     for (int i = 0; i < 225; i++) {
         leftside[i].x = 1;
         rightside[i].x = CamW - 1;
         if(i < 120){
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
     startlostleftside = 0;
     lostleftside = 0;
     reallostrightside = 0;
     startlostrightside = 0;
     lostrightside = 0;
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
   
    for (; i <= 225; i++) {//主循环，左右边界同时使用这个for循环

        if (leftflag) {//左边巡线
            //由上一个结构体点得到当前结构体点
            leftside[i] = Eightsearch(leftnowpos, leftside[i - 1], LEFT);
            
            length_leftside++;

            if(lostleft_pos_end > leftside[i].y)
                lostleft_pos_end = leftside[i].y;

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

        }if (rightflag) {//右边巡线，以此类推
            
            rightside[i] = Eightsearch(rightnowpos, rightside[i - 1], RIGHT);

            length_rightside++;

            if(lostright_pos_end > rightside[i].y)
                lostright_pos_end = rightside[i].y;

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
            if(midleft[i] < 3)
            reallostleftside++;

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
    for (j = 35; j <= 180; j++)//优化
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
        if(LeftBound[searchmid].y<75)
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
                            }else if(midx < (double)temp.orgpoint.x && midy < (double)temp.orgpoint.y){   
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
        if(RightBound[searchmid].y<75)
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
                            }else if(midx > (double)temp.orgpoint.x && midy < (double)temp.orgpoint.y){
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



std::string ElementString[17] = {
    "Normal",
    "PreCross",
    "PreRightRound","InRightRound","OutRightRound","AcrossRightRound",
    "PreLeftRound","InLeftRound","OutLeftRound","AcrossLeftRound",
    "ZebraFlag",
    "RecMaintenWay","PreMainten","InMainten","OutMainten",
    "PreBoom","YellowHinder"};//特殊元素主状态标志位显示字符串
enum ElementFlag elementflag = Normal;//主状态主标志位
std::string ControSideFlagString[4] = {"MidSide","RightSide","LeftSide","Straight"};
enum ControSideFlag controsideflag = MidSide;//依赖左右边界巡线主标志位
//打角行一定情况下动态，最大不超过75
uint8 waycontrolline1 = 42;//第一打角行42
uint8 waycontrolline2 = 43;//第二打角行43
uint8 realcontrolline1 = 42;//动态打角行42
uint8 realcontrolline2 = 43;//动态打角行43
uint8 Sidepianyi = 43;//左右巡线偏移量

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
/**************************************************************************************************************************/
//十字部分处理
void MakePreCrossLine(void){
    if(RightAngelList.angelnum == 0 || LeftAngelList.angelnum == 0)
        return;
    
    if(lostleftside > 8 &&lostrightside > 8){
        if(!RightAngelList.typeOneList.empty() &&!RightAngelList.typeTwoList.empty() &&RightAngelList.typeThreeList.size() > 1
         &&!LeftAngelList.typeOneList.empty() &&!LeftAngelList.typeTwoList.empty() &&LeftAngelList.typeThreeList.size() > 1){
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
            return;
        }
    }
    if(reallostleftside > 15 &&reallostrightside > 15){
        if((RightAngelList.typeOneList.empty() || RightAngelList.typeOneList[0].orgpoint.y<45) &&!RightAngelList.typeTwoList.empty()
         &&(LeftAngelList.typeOneList.empty() || LeftAngelList.typeOneList[0].orgpoint.y<45) &&!LeftAngelList.typeTwoList.empty()){
            
            int x1 = RightAngelList.typeTwoList[0].orgpoint.x;
            int y1 = RightAngelList.typeTwoList[0].orgpoint.y;
            if(RightAngelList.typeTwoList[0].num + 2 > length_rightside)
                return;
            int x2 = rightside[RightAngelList.typeTwoList[0].num + 2].x;
            int y2 = rightside[RightAngelList.typeTwoList[0].num + 2].y;
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

            x1 = LeftAngelList.typeTwoList[0].orgpoint.x;
            y1 = LeftAngelList.typeTwoList[0].orgpoint.y;
            if(LeftAngelList.typeTwoList[0].num + 2 > length_leftside)
                return;
            x2 = leftside[LeftAngelList.typeTwoList[0].num + 2].x;
            y2 = leftside[LeftAngelList.typeTwoList[0].num + 2].y;
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
            return;
        }
    }
}
// void FindCrossTopChangePoint(void){
//     for(int i = 60;i > lostleft_pos_end;i--){
//         if(midleft[i] - midleft[i + 3] >= 30 && abs(midleft[i - 1] - midleft[i]) <= 5){
//             leftchangepoint[1].x = midleft[i];
//             leftchangepoint[1].y = i;
//             FindLeftCPointCross = true;
//         }
//     }
//     for(int i = 60;i > lostright_pos_end;i--){
//         if(midright[i] - midright[i + 3] <= -30 && abs(midright[i - 1] - midright[i]) <= 5){
//             rightchangepoint[1].x = midright[i];
//             rightchangepoint[1].y = i;
//             FindRightCPointCross = true;
//         }
//     }
// }
// void RecPreCross(void){
//     if(lostleftside > 25 && lostrightside > 25 && length_leftside > 125 && length_rightside > 125 
//     &&findleftflag_0 &&FindLeftCPointCross &&findrightflag_0 &&FindRightCPointCross){
//     elementflag = PreCross;
//     MakePreCrossLine();
//     }
// }
/**************************************************************************************************************************/
//识别斑马线部分
void recozebra(void){
    if(reallostleftside < 3 && reallostrightside < 3)
        return;
    else if(reallostleftside > 20 && reallostrightside > 20)
        return;


    int colorcount = 0;
    int sumcount = 0;
    for(uint8 line = waycontrolline1  - 5; line <= waycontrolline1 + 20;line++){

        for(int a = 115;a >= 35;a--){
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
    if(lostleftside <= 4 && reallostleftside <= 10 && lostrightside > 18 && lostleft_pos_end < 20 && length_rightside > 190
    &&dofhead > 70){
        if(RightAngelList.typeOneList.size()>0 &&RightAngelList.typeOneList[0].orgpoint.angle>90
        &&RightAngelList.typeThreeList.size()>0
        &&(LeftAngelList.typeOneList.size() == 0 || LeftAngelList.typeOneList[0].orgpoint.y<20)){
            elementflag = PreRightRound;
            cout<<"turn into PreRightRound!"<<endl;
        }
    }
}
void RecInRightRound(void){
    if(lostleftside <= 4 &&lostrightside > 12 &&length_rightside > 140 
    &&RecInRoundConer(0) &&dofhead > 60){
        if(RightAngelList.typeTwoList.size()>0 &&RightAngelList.typeTwoList[0].orgpoint.y<45 
        &&RightAngelList.typeTwoList[0].orgpoint.angle<90 &&RightAngelList.typeThreeList.size()>0 
        &&(LeftAngelList.typeOneList.size() == 0 || LeftAngelList.typeOneList[0].orgpoint.y<30)){
            OutPreRightRoundFlag = true;
        }
    }
}
void RecOutRightRound(void){
    if(lostleftside >= 10 &&reallostrightside >= 20 &&dofhead > 50){
        if(LeftAngelList.typeOneList.size()>0 &&LeftAngelList.typeOneList[0].orgpoint.angle>90){
            elementflag = OutRightRound;
            cout<<"turn into OutRightRound!"<<endl;
        }
    }
}
void RecAcrossrightRound(void){
    if(lostleftside <= 4 &&reallostleftside <= 25 && reallostrightside >= 16
    && dofhead > 65 && SLeftk < 12 && SLeftk > 0 && LeftMeank < -0.1){
        if(RightAngelList.typeTwoList.size()>0 &&RightAngelList.typeTwoList[0].orgpoint.y<45 
        &&RightAngelList.typeTwoList[0].orgpoint.angle<90 &&RightAngelList.typeThreeList.size()>0 
        &&(LeftAngelList.typeOneList.size() == 0 || LeftAngelList.typeOneList[0].orgpoint.y<30)){
            elementflag = AcrossRightRound;
            cout<<"turn into AcrossRightRound!"<<endl;
        }
    }
}
/**************************************************************************************************************************/
//左环岛处理
void RecPreLeftRound(void){
    if(lostleftside > 18 &&lostrightside <= 4 && reallostrightside <= 10 &&length_leftside > 190 &&lostright_pos_end < 20
    &&dofhead > 70){
        if(LeftAngelList.typeOneList.size()>0 &&LeftAngelList.typeOneList[0].orgpoint.angle>90
        &&LeftAngelList.typeThreeList.size()>0
        &&(RightAngelList.typeOneList.size() == 0 || RightAngelList.typeOneList[0].orgpoint.y<20)){
            elementflag = PreLeftRound;
            cout<<"turn into PreLeftRound!"<<endl;
        }
    }
}
void RecInLeftRound(void){
    if(lostleftside > 12 &&lostrightside <= 4 &&length_leftside > 140 &&RecInRoundConer(1) &&dofhead > 60){
        if(LeftAngelList.typeTwoList.size()>0 &&LeftAngelList.typeTwoList[0].orgpoint.y<45 
        &&LeftAngelList.typeTwoList[0].orgpoint.angle<90 &&LeftAngelList.typeThreeList.size()>0 
        &&(RightAngelList.typeOneList.size() == 0 || RightAngelList.typeOneList[0].orgpoint.y<30)){
            OutPreLeftRoundFlag = true;
        }
    }
}
void RecOutLeftRound(void){
    if(reallostleftside >= 20 &&lostrightside > 10 &&dofhead > 50){
        if(RightAngelList.typeOneList.size()>0 &&RightAngelList.typeOneList[0].orgpoint.angle>90){
            elementflag = OutLeftRound;
            cout<<"turn into OutLeftRound!"<<endl;
        }
    }
}
void RecAcrossLeftRound(void){
    if(reallostleftside >= 16 && lostrightside <= 4 && reallostrightside <= 25
    && dofhead > 65 && SRightk < 12 && SRightk > 0 && RightMeank < 1 && RightMeank > 0.1){
        if(LeftAngelList.typeTwoList.size()>0 &&LeftAngelList.typeTwoList[0].orgpoint.y<45 
        &&LeftAngelList.typeTwoList[0].orgpoint.angle<90 &&LeftAngelList.typeThreeList.size()>0 
        &&(RightAngelList.typeOneList.size() == 0 || RightAngelList.typeOneList[0].orgpoint.y<30)){
        elementflag = AcrossLeftRound;
        cout<<"turn into AcrossLeftRound!"<<endl;
        }
    }
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
void RecYellowCone(Mat src){
    Mat Cone_mat = cv::Mat(CamH,CamW,CV_8UC1);
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
   
    for(int i = 0; i < src.rows; ++i){
        cv::Vec3b *pixel = src.ptr<cv::Vec3b>(i); // point to first pixel in row
        for(int j = 0; j < src.cols; ++j){

        // 绿-蓝
            int diff1 = pixel[j][1] - (int)pixel[j][0] * 1.1; //省赛1.1
            if (diff1 < 0) 
                diff1 = 0;
            else if(diff1 > 2)
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
int32 OutZebraCount = 0;
int SumZebraCount = 1;//一过斑马线而不入
int ZebraGuai = 80;
bool ZebraEnable = true;
bool LeftRoundEnable = true;
bool RightRoundEnable = true;
bool RedEnable = true;
int32 CountZebraEnable = 0;
int32 CountBreakRed = 0;
bool MaintenWay = true;//维修区方向标志位true往左拐，false往右拐
int MaintenCountPatient = 0;
int MaintenCountTumble = 0;
int MaintenCountEvil = 0;
int MaintenCountThief = 0;
int CountHaveCone = 0;
bool InMaintenFlag = false;
bool MaintenEnable = true;
int BombCount = 0;
int YellowHinderCount = 0;
bool YellowHinderWay = true;//危险区寻边方向，true代表锥桶在左边寻右边线，false寻左边线
bool HinderWayMetux = false;
/**************************************************************************************************************************/
void ReconElements(void){

    for(int i = 0;i < 13;i++){
        detectTypeNum[i] = 0;
    }

    //conelist.clear();
    if(!Tr_result.empty()){
        for(TrDetectResult i : Tr_result){

            detectTypeNum[i.type]++;
                    
            // if(i.type == 3)
            //      conelist.push_back(i);
        }
    }

    switch(elementflag)
    {
        case Normal:

            

            if(CountZebraEnable < 300){
                CountZebraEnable++;
            }else if(CountZebraEnable >= 300){
                recozebra();
            }

            // if(findleftflag_0 &&findrightflag_0 && lostleftside >= 25 && lostrightside >= 25){
            //     FindLeftCPointCross = false;
            //     FindRightCPointCross = false;
            //     FindCrossTopChangePoint();
            //     RecPreCross();
            // }

            if(RightRoundEnable)
                RecPreRightRound();
            if(LeftRoundEnable)
                RecPreLeftRound();

            if(detectTypeNum[8] >= 1 && detectTypeNum[12] == 0 && detectTypeNum[6] == 0 && detectTypeNum[11] == 0){
                MaintenCountPatient++;
                MaintenCountTumble = 0;
                MaintenCountEvil = 0;
                MaintenCountThief = 0;
                if(MaintenCountPatient >= 3 &&MaintenEnable){
                    elementflag = RecMaintenWay;//识别到病人patient，左拐
                    MaintenWay = true;
                    MaintenCountPatient = 0;
                    cout<<"Left turn into RecMaintenWay!"<<endl;
                }
            }else if(detectTypeNum[8] == 0 && detectTypeNum[12] >= 1 && detectTypeNum[6] == 0 && detectTypeNum[11] == 0){
                MaintenCountTumble++;
                MaintenCountPatient = 0;
                MaintenCountEvil = 0;
                MaintenCountThief = 0;
                if(MaintenCountTumble >= 3 &&MaintenEnable){
                    elementflag = RecMaintenWay;//识别到摔倒tumble，左拐
                    MaintenWay = true;
                    MaintenCountTumble = 0;
                    cout<<"Left turn into RecMaintenWay!"<<endl;
                }
            }else if(detectTypeNum[8] == 0 && detectTypeNum[12] == 0 && detectTypeNum[6] >= 1 && detectTypeNum[11] == 0){
                MaintenCountEvil++;
                MaintenCountPatient = 0;
                MaintenCountTumble = 0;
                MaintenCountThief = 0;
                if(MaintenCountEvil >= 3 &&MaintenEnable){
                    elementflag = RecMaintenWay;//识别到歹徒evil，右拐
                    MaintenWay = false;
                    MaintenCountEvil = 0;
                    cout<<"Right turn into RecMaintenWay!"<<endl;
                }
            }else if(detectTypeNum[8] == 0 && detectTypeNum[12] == 0 && detectTypeNum[6] == 0 && detectTypeNum[11] >= 1){
                MaintenCountThief++;
                MaintenCountPatient = 0;
                MaintenCountTumble = 0;
                MaintenCountEvil = 0;
                if(MaintenCountThief >= 3 &&MaintenEnable){
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

            if(detectTypeNum[0] >= 1){
                
                if(BombCount >=3){
                    elementflag = PreBoom;
                    cout<<"turn into PreBoom!"<<endl;
                    BombCount = 0;
                }else if(BombCount >= 0 && BombCount < 3)
                    BombCount++;
            }

            MakePreCrossLine();

            waycontrolline1 = 42;//第一打角行
            waycontrolline2 = 43;//第二打角行

            controsideflag = MidSide;

        break;
        case PreCross:
            //MakePreCrossLine();
            controsideflag = MidSide;
            if(OutCrossCount < 25){
                OutCrossCount++;
            }else if(OutCrossCount == 25){
                elementflag = Normal;
                OutCrossCount = 0;
            }

        break;
        case PreRightRound:
            
            Sidepianyi = 46;
            controsideflag = LeftSide;
            RecInRightRound();
            if(OutPreRightRoundCount < 2)
            OutPreRightRoundCount++;
            else if(OutPreRightRoundFlag && OutPreRightRoundCount >= 2){
            OutPreRightRoundCount = 0;
            OutPreRightRoundFlag = false;
            cout<<"turn into InRightRound!"<<endl;
            elementflag = InRightRound;
            }
            //以下为鲁棒性部分
            if(OutPreRound < 150){
                OutPreRound++;
            }else if(OutPreRound == 150){
                OutPreRound = 0;
                elementflag = Normal;
            }
        break;
        case InRightRound:
            OutPreRound = 0;
            //Sidepianyi = 32;//
            
            if(OutOutRightRoundCount < 40){//原为120帧************************************************************
            OutOutRightRoundCount++;
            controsideflag = RightSide;
            }else{
                if(length_rightside > 80 && length_leftside > 115 &&lostleftside == 0&& dofhead >= 60){
                    controsideflag = MidSide;
                }
                RecOutRightRound();
            }
            waycontrolline1 = 54;//第一打角行
            waycontrolline2 = 55;//第二打角行
           
            
        break;
        case OutRightRound:
            controsideflag = RightSide;
            RoadK(1,90,50);
            RecAcrossrightRound();
            if(OutRoundCount < 200){
                OutRoundCount++;
            }else if(OutRoundCount == 200){
                OutRoundCount = 0;
                elementflag = Normal;
                RightRoundEnable = false;
            }
        break;
        case AcrossRightRound:
            OutRoundCount = 0;
            Sidepianyi = 40;
            controsideflag = LeftSide;//
            //退出标志位计时
            if(OutAcrossrightRoundCount < 15)
            OutAcrossrightRoundCount++;
            else{
            OutAcrossrightRoundCount = 0;
            elementflag = Normal;
            RightRoundEnable = false;
            }
           
        break;
        case PreLeftRound:
            Sidepianyi = 46;
            controsideflag = RightSide;
            RecInLeftRound();
            if(OutPreLeftRoundCount < 2)
            OutPreLeftRoundCount++;
            else if(OutPreLeftRoundFlag && OutPreLeftRoundCount >= 2){
            OutPreLeftRoundCount = 0;
            OutPreLeftRoundFlag = false;
            cout<<"turn into InLeftRound!"<<endl;
            elementflag = InLeftRound;
            }
            //以下为鲁棒性部分
            if(OutPreRound < 150){
                OutPreRound++;
            }else if(OutPreRound == 150){
                OutPreRound = 0;
                elementflag = Normal;
            }
        break;
        case InLeftRound:
            OutPreRound = 0;
            //Sidepianyi = 34;//
            
            if(OutOutLeftRoundCount < 40){//原为120帧
                OutOutLeftRoundCount++;
                controsideflag = LeftSide;
            }else{
                if(length_leftside > 80 && length_rightside > 115 &&lostrightside == 0&& dofhead >= 60){
                    controsideflag = MidSide;
                }
                RecOutLeftRound();
            }
            waycontrolline1 = 54;//第一打角行
            waycontrolline2 = 55;//第二打角行
           
        break;
        case OutLeftRound:
            //Sidepianyi = 28;//
            controsideflag = LeftSide;
            RoadK(0,90,45);
            RecAcrossLeftRound();
            if(OutRoundCount < 200){
                OutRoundCount++;
            }else if(OutRoundCount == 200){
                OutRoundCount = 0;
                elementflag = Normal;
                LeftRoundEnable = false;
            }
            //waycontrolline1 = 54;//第一打角行
            //waycontrolline2 = 55;//第二打角行
        break;
        case AcrossLeftRound:
            OutRoundCount = 0;
            Sidepianyi = 40;
            controsideflag = RightSide;//
            //退出标志位计时
            if(OutAcrossleftRoundCount < 15)
            OutAcrossleftRoundCount++;
            else{
            OutAcrossleftRoundCount = 0;
            elementflag = Normal;
            LeftRoundEnable = false;
            }
            
        break;
        case ZebraFlag:
            //退出标志位计时
           
            controsideflag = MidSide;

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
                if(i.y > 50 && i.x > 42 && i.x < 103){
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
                    if(conelist[0].x <= Mid)
                        YellowHinderWay = true;
                    else if(conelist[0].x > Mid)
                        YellowHinderWay = false;

                    elementflag = YellowHinder;
                    YellowHinderCount = 0;
                    cout<<"turn into YellowHinder"<<endl;
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

                if(HinderWayMetux && conelist[0].y > 75){
                    HinderWayMetux = false;
                    if(YellowHinderWay){
                        YellowHinderWay = false;
                    }else{
                        YellowHinderWay = true;
                    }
                    cout<<"Have changed the way:"<<YellowHinderWay<<endl;
                }

                if(conelist[0].y < 45){
                    HinderWayMetux = true;
                }
                
            }

            if(YellowHinderWay){
                Sidepianyi = 30;
                controsideflag = RightSide;
            }else{
                Sidepianyi = 30;
                controsideflag = LeftSide;
            }

            if(!LeftAngelList.typeThreeList.empty()&&!LeftAngelList.typeTwoList.empty())
                if(LeftAngelList.typeThreeList[0].orgpoint.y > 65 &&LeftAngelList.typeTwoList[0].orgpoint.y > 65){
                    elementflag = Normal;
                    cout<<"turn into Normal"<<endl;
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

  if(waycontrolline1 < Topline){
    realcontrolline1 = Topline + 2;
    realcontrolline2 = realcontrolline1 + 1;

    datak = (double)((Topline - waycontrolline1)/10)*1.1 + datak;//Topline - waycontrolline1 大约在0到40之间
  }else{
    realcontrolline1 = waycontrolline1;
    realcontrolline2 = waycontrolline2;

  }

  if(realcontrolline2 > CamH - 15){
    realcontrolline1 = CamH - 15;
    realcontrolline2 = realcontrolline1 + 1;
  }
    
  control = (((int)midline[realcontrolline1 ] - Mid)*0.6 + 
                ((int)midline[realcontrolline2 ] - Mid)*0.4)*datak;

  switch(elementflag)
    {
        case ZebraFlag:
            motor = 0;
        break;
        case InRightRound:
            motor = 40;
        break;
        case PreRightRound:
            sendlist[0] = 1;
            motor = 40;
        break;
        case OutRightRound:
            motor = 40;
        break;
        case InLeftRound:
            motor = 40;
        break;
        case PreLeftRound:
            sendlist[1] = 1;
            motor = 40;
        break;
        case OutLeftRound:
            motor = 40;
        break;
        case PreMainten:
            motor = 40;
        break;
        case InMainten:

            if(MaintenWay)
                control = -29;
            else
                control = 29;

            motor = 25;
        break;
        case OutMainten:

            if(MaintenWay)
                control = -29;
            else
                control = 29;

            motor = -15;
        break;
        case YellowHinder:

            motor = 40;

        break;
        default:

            if(dofhead >= 60 && lostleftside < 8 && lostrightside < 8){
                SpeedCount++;
                if(SpeedCount > 11){
                    SpeedCount = 11;
                }
            }else
                SpeedCount = 0;

            if(dofhead >= 60 && motor != 0){//当车头距离边界的距离在85到90之间，速度对应为60到80
                if(dofhead > 90){
                    dofhead = 90;
                }
                if(SpeedCount >= 10)
                    motor = SH + SHR * (dofhead - 60) / 30;
                else
                    motor = SM + SMR;
            }else if(dofhead < 40 && dofhead >= 30)
                motor = SM + SMR * (dofhead - 30) / 10;
            else if(dofhead < 30)
                motor = SL;
        break;

    }
}









