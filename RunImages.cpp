#include "src/headfile.hpp"
#include "src/MyUart.hpp"
#include "src/mycapture.hpp"
#include "src/detection.hpp"
#include <linux/serial.h>
#include <signal.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ctime>

using namespace std;
using namespace cv;

#define CV_COLOR_RED cv::Scalar(0, 0, 255)   //纯红
#define CV_COLOR_GREEN cv::Scalar(0, 255, 0) //纯绿
#define CV_COLOR_BLUE cv::Scalar(255, 0, 0)  //纯蓝

#define CV_COLOR_DARKGRAY cv::Scalar(169, 169, 169) //深灰色
#define CV_COLOR_DARKRED cv::Scalar(0, 0, 139)      //深红色
#define CV_COLOR_ORANGERED cv::Scalar(0, 69, 255)   //橙红色

#define CV_COLOR_CHOCOLATE cv::Scalar(30, 105, 210) //巧克力
#define CV_COLOR_GOLD cv::Scalar(10, 215, 255)      //金色
#define CV_COLOR_YELLOW cv::Scalar(0, 255, 255)     //纯黄色

#define CV_COLOR_OLIVE cv::Scalar(0, 128, 128)        //橄榄色
#define CV_COLOR_LIGHTGREEN cv::Scalar(144, 238, 144) //浅绿色
#define CV_COLOR_DARKCYAN cv::Scalar(139, 139, 0)     //深青色

#define CV_COLOR_SKYBLUE cv::Scalar(230, 216, 173) //天蓝色
#define CV_COLOR_INDIGO cv::Scalar(130, 0, 75)     //藏青色
#define CV_COLOR_PURPLE cv::Scalar(128, 0, 128)    //紫色

#define CV_COLOR_PINK cv::Scalar(203, 192, 255)    //粉色
#define CV_COLOR_DEEPPINK cv::Scalar(147, 20, 255) //深粉色
#define CV_COLOR_VIOLET cv::Scalar(238, 130, 238)  //紫罗兰

#define CV_COLOR_BLACK cv::Scalar(0, 0, 0)  //黑色

void draw_line(cv::Mat frame, uint8 start_x, uint8 start_y, uint8 end_x, uint8 end_y, cv::Scalar CV_COLOR) {
    cv::Point start = cv::Point(start_x, start_y); //直线起点
    cv::Point end = cv::Point(end_x, end_y);       //直线终点
    cv::line(frame, start, end, CV_COLOR);
}

void draw_point(cv::Mat frame, uint8 x, uint8 y, cv::Scalar CV_COLOR) {
    cv::Point point = cv::Point(x, y);
    cv::circle(frame, point, 1, CV_COLOR,2);
}

// 以中心点坐标和半径画圆
void draw_circle(cv::Mat frame, uint8 x, uint8 y, uint8 r, cv::Scalar CV_COLOR) {
    cv::Point point = cv::Point(x, y);
    cv::circle(frame, point, r, cv::Scalar(0, 0, 255));
}


int controldata = 0;//向下位机发送的打角行数据
int motordata = 28;//向下位机发送电机控制数据
uint8 sendmessage[14] = {0};//向下位机发送的数组
Mat m1;
Mat datashow(CamH,CamW,CV_8UC3,Scalar(0,255,0));//显示数据
void VisionData(String data,int x,int y);
void MyCVMatTounint8arry(Mat m1);
void Vision(void);
void getcontroldata_main(int SH,int SHR,int SM,int SMR,int SL);
//bool MakeLineFlag = false;

pthread_mutex_t ai_lock;
pthread_cond_t ai_sig;
bool ai_flag, ai_finish_flag;
static void *ai_buf[60000];
int ai_len;
int ai_show_pos = -1;
int show_pos = -1;

std::vector<PredictResult> results; //AI推理结果

void* ai_thread(void* args)//AI线程
{
    //bool ret;不知道为什么这个没用上

    PPNCDetection detection;
    if (!detection.init("../res_datav1/model/yolov3_mobilenet_v1")) //AI推理初始化
        //return 1;

    printf("compiler passed");
    while(true)
    {
        // 等待主线程通知运行
        pthread_mutex_lock(&ai_lock);
        ai_finish_flag = true;
        while (ai_flag != true)
            // ref: https://www.cnblogs.com/zhangxuan/p/6526854.html
            pthread_cond_wait(&ai_sig, &ai_lock);
        ai_flag = false;
        pthread_mutex_unlock(&ai_lock);

        cv::Mat Aiframe(cv::Size(320, 240), CV_8UC3);//CAP_WIDTH,CAP_HEIGHT

        cv::imdecode(cv::Mat(1, ai_len, CV_8U, ai_buf), cv::IMREAD_COLOR, &Aiframe);

        // cv::resize(Aiframe, Aiframe, cv::Size(180, 140), 0, 0, cv::INTER_LINEAR);
        //Aiframe = Aiframe(cv::Rect(0, CAP_HEIGHT * 0.25, CAP_WIDTH, CAP_HEIGHT*0.75));

        cv::Point rookPoints[1][4];
        rookPoints[0][0] = cv::Point(0, 0);//左上角
        rookPoints[0][1] = cv::Point(319, 0);//右上角
        rookPoints[0][2] = cv::Point(319, 240 * 0.05);//右下角
        rookPoints[0][3] = cv::Point(0, 240 * 0.05);//左下角
        int npt[]={4};//二维数组 每列长度
        const cv::Point* ppt[1] = {rookPoints[0]};//所有多边形点坐标
        fillPoly(Aiframe, ppt, npt, 1, cv::Scalar(0, 0, 0));

        auto feeds = detection.preprocess(Aiframe, {320, 320});
        detection.run(*feeds);
        detection.render();  // get results

        // cv::Mat imageAi = Aiframe.clone();
        detection.drawBox(Aiframe);

        //my_image_show("AIimg", Aiframe, show_scale, ai_show_pos);//AI窗口
       
        namedWindow("AiImage", WINDOW_NORMAL);
        imshow("AiImage",Aiframe);//
        
    }
}

static void *buf[60000];
int len;

bool ai_end_frame = false;

void ai_get_data_back() {
    ai_end_frame = false;
    pthread_mutex_lock(&ai_lock);
    if (ai_finish_flag == true) {
        ai_finish_flag = false;

        // ====================== handle data
        // 取回数据
        results.empty();
        results.assign(ai_results.begin(), ai_results.end());      

        ai_end_frame = true;
        
        // 灌入新数据
        memcpy(ai_buf, buf, len);
        ai_len = len;

        ai_show_pos = show_pos;
        // ====================== handle data end
        
        ai_flag = true;
        pthread_cond_signal(&ai_sig);
    }
    pthread_mutex_unlock(&ai_lock);
}




int main(int argc, char *argv[]) {

  int VisionEdge = 0;
  cout<<"输入1开启传统边界可视化：";
  cin>>VisionEdge;
  if(VisionEdge == 1)
    MakeLineFlag = true;

  
  /*****************************************************/
  //声明uart通信
  //usb_uart_recv_init();//usb_uart接受初始化函数
  //usb_uart_send_init();//usb_uart发送初始化函数
  //int fd;
  //int wlen;
  //fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
  //set_interface_attribs(fd, B460800);  
  //set_mincount(fd, 0);


	unsigned char *yuv422frame = NULL;
	unsigned long yuvframeSize = 0;
  CvMat cvmat;
  IplImage*imgs;

 

	string videoDev="/dev/video0";
	V4L2Capture *vcap = new V4L2Capture(const_cast<char*>(videoDev.c_str()),
			320, 240);
	vcap->openDevice();
	vcap->initDevice();
	vcap->startCapture();

/*****************************************************/
  //计时声明
  //clock_t begin,end;
  Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);

  while(1){
  
  
	vcap->getFrame((void **) &yuv422frame, (size_t *)&yuvframeSize);
  cvmat = cvMat(240,320,CV_8UC3,(void*)yuv422frame);	
  imgs = cvDecodeImage(&cvmat,1);
  m1 = cvarrToMat(imgs);

  //begin = clock();
  resize(m1, m1, Size(CamW, CamH));

  cv::Mat frame_splited[3];
  cv::split(m1, frame_splited);
  cv::Mat red_ch = frame_splited[2].clone();
  cv::blur(red_ch, red_ch, cv::Size(3, 3)); // 减少噪点

  memcpy(images, red_ch.isContinuous() ? red_ch.data : red_ch.clone().data, sizeof images); // Mat 转数组

  Mat Cone_mat = cv::Mat(CamH,CamW,CV_8UC1);
   
  for(int i = 0; i < m1.rows; ++i){
    cv::Vec3b *pixel = m1.ptr<cv::Vec3b>(i); // point to first pixel in row
      for(int j = 0; j < m1.cols; ++j){

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
  namedWindow("cone_dilate", WINDOW_NORMAL);
  imshow("cone_dilate", Cone_mat);
  vector<vector<Point>> contours_cone;
  vector<Rect> rectlist_cone;//锥桶识别结果存储列表
  findContours(Cone_mat, contours_cone, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  for (vector<Point> j : contours_cone) {
      Rect result = boundingRect(j);
      if(result.area() < 40)
        continue;
      // if(result.width > 9 && result.height > 9 && (result.y + result.height/2) > 24){
      //   rectlist_cone.push_back(result);
      // }
      draw_point(m1,result.x + result.width/2,result.y + result.height/2,CV_COLOR_GOLD);
      rectangle(m1,result,CV_COLOR_RED,1);
  }
  namedWindow("img", WINDOW_NORMAL);
    imshow("img", m1);

  Threshold = otsuThreshold(images[0], CamW, CamH);
  Binaryisation(Threshold);

  Find_FengDing();//注意该函数必须在前面运行

  InitialiseFlag();

  scanbotton();

  getroadside();

  getmidline();

  getdistance();

  getlostside();

  ReconElements();

  getmidline();

  getcontroldata_main(55,10,45,10,42);

  //end = clock();
  //cout<<"跑完一帧所用时间:"<<double(end - begin) / CLOCKS_PER_SEC * 1000<<"ms"<<endl;

  

  Vision();
  waitKey(1);
 

 // usb_uart_5byte_send_3_0(fd, send, 30,-10);
  
  cvReleaseImage(&imgs);
  vcap->backFrame();
  
   


  }


	vcap->stopCapture();
	vcap->freeBuffers();
	vcap->closeDevice();
  
  return 0;

}


/*************************************************************************************************************************************/
void MyCVMatTounint8arry(Mat m1){
  for(int h = 0;h < CamH;h++){
    for(int w = 0;w < CamW;w++){
      images[h][w] = m1.at<uchar>(h,w);
    }
  }
}
/*************************************************************************************************************************************/
void VisionData(String data,int x,int y){
  putText(datashow,data,Point(x,y),FONT_HERSHEY_COMPLEX,0.3,Scalar(255,0,0),1,8);
}
/*************************************************************************************************************************************/
void Vision(){

  datashow = Mat(CamH*2.5,CamW,CV_8UC3,Scalar(0,255,0));
  cv::Mat vision = cv::Mat(CamH,CamW,CV_8UC1,images);
  cv::Mat rgbvision = cv::Mat(CamH,CamW,CV_8UC3);
  cvtColor(vision,rgbvision,CV_GRAY2BGR);
  

//   String yuzhi = to_string(Threshold);
  //VisionData(yuzhi,20,20);
   VisionData("dofhean:"+to_string(dofhead),20,20);
   VisionData("length_left:"+to_string(length_leftside),20,32);
   VisionData("length_right:"+to_string(length_rightside),20,44);
   VisionData("lostleftside:"+to_string(lostleftside),20,56);
   VisionData("lostrightside:"+to_string(lostrightside),20,68);
   VisionData("elementflag:"+ElementString[(int)elementflag],20,80);
   VisionData("controldata:"+to_string(controldata),20,92);

   VisionData("reallostleftside:"+to_string(reallostleftside),20,152);
   VisionData("reallostrightside:"+to_string(reallostrightside),20,164);
   VisionData("LeftMeank:"+to_string(LeftMeank),20,176);
   VisionData("SLeftk:"+to_string(SLeftk),20,188);
   VisionData("Topline:"+to_string(Topline),20,200);

   for(angelpoint an : LeftAngelList.List){
    if(an.angletype == 1)
      draw_point(rgbvision,an.orgpoint.x,an.orgpoint.y,CV_COLOR_CHOCOLATE);
    else if(an.angletype == 2)
      draw_point(rgbvision,an.orgpoint.x,an.orgpoint.y,CV_COLOR_GOLD);
    else
      draw_point(rgbvision,an.orgpoint.x,an.orgpoint.y,CV_COLOR_YELLOW);
    }
   for(angelpoint an : RightAngelList.List){
    if(an.angletype == 1)
      draw_point(rgbvision,an.orgpoint.x,an.orgpoint.y,CV_COLOR_VIOLET);
    else if(an.angletype == 2)
      draw_point(rgbvision,an.orgpoint.x,an.orgpoint.y,CV_COLOR_DEEPPINK);
    else
      draw_point(rgbvision,an.orgpoint.x,an.orgpoint.y,CV_COLOR_PINK);
    }


  if(!MakeLineFlag){
      //显示八邻域左边线
      for(int i = 0;i < length_leftside;i++){
        rgbvision.at<Vec3b>(leftside[i].y,leftside[i].x)[0] = 255;
        rgbvision.at<Vec3b>(leftside[i].y,leftside[i].x)[1] = 0;
        rgbvision.at<Vec3b>(leftside[i].y,leftside[i].x)[2] = 0;
      }
      //显示八邻域右边线
      for(int i = 0;i < length_rightside;i++){
        rgbvision.at<Vec3b>(rightside[i].y,rightside[i].x)[0] = 255;
        rgbvision.at<Vec3b>(rightside[i].y,rightside[i].x)[1] = 0;
        rgbvision.at<Vec3b>(rightside[i].y,rightside[i].x)[2] = 255;
      }
    }else {
      for(int i = 0;i < CamH - 2;i++){
        rgbvision.at<Vec3b>(i,midleft[i])[0] = 255;//蓝色
        rgbvision.at<Vec3b>(i,midleft[i])[1] = 0;
        rgbvision.at<Vec3b>(i,midleft[i])[2] = 0;
        rgbvision.at<Vec3b>(i,midright[i])[0] = 255;//红色
        rgbvision.at<Vec3b>(i,midright[i])[1] = 0;
        rgbvision.at<Vec3b>(i,midright[i])[2] = 255;
      }
    //MakeLineFlag = false;
    }

  for (int i = CamH - 4; i > 2; i--) {
      rgbvision.at<Vec3b>(i,midline[i])[0] = 0;
      rgbvision.at<Vec3b>(i,midline[i])[1] = 255;
      rgbvision.at<Vec3b>(i,midline[i])[2] = 0;
   }
  rgbvision.at<Vec3b>(waycontrolline1,Mid)[0] = 0;
  rgbvision.at<Vec3b>(waycontrolline1,Mid)[1] = 0;
  rgbvision.at<Vec3b>(waycontrolline1,Mid)[2] = 255;
  rgbvision.at<Vec3b>(waycontrolline2,Mid)[0] = 0;
  rgbvision.at<Vec3b>(waycontrolline2,Mid)[1] = 0;
  rgbvision.at<Vec3b>(waycontrolline2,Mid)[2] = 255;
    // for(int i = 0;i < 50;i++){//改为蓝色测试
    // rgbvision.at<Vec3b>(i,2)[0] = 255;
    // rgbvision.at<Vec3b>(i,2)[1] = 0;
    // rgbvision.at<Vec3b>(i,2)[2] = 0;
    // }
  namedWindow("GrayImage", WINDOW_NORMAL);
  imshow("GrayImage", rgbvision);

  namedWindow("Data",WINDOW_NORMAL);
  imshow("Data",datashow);
  
}
void getcontroldata_main(int SH,int SHR,int SM,int SMR,int SL){

  getcontroldata(sendmessage,controldata,motordata,SH,SHR,SM,SMR,SL);

}