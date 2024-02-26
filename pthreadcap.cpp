#include "src/headfile.hpp"
#include "src/detection.hpp"
#include "src/json.hpp"
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

#include <dirent.h>
#include <sys/stat.h>

#include <unistd.h>



extern "C" {
#include "src/cap.h"
#include "src/comm.h"
}

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

const int show_scale = 2;
int show_pos = -1;

// my imshow
void my_image_show(const char *name, const cv::Mat &mat, int show_scale_ = show_scale, int my_show_pos = -1) {
    // cv::namedWindow(name, cv::WINDOW_NORMAL);
    cv::Mat buf;
    if (strcmp(name, "perspective") == 0) {
        cv::resize(mat, buf, cv::Size(mat.cols * show_scale / 2, mat.rows * show_scale / 2), 0, 0, cv::INTER_NEAREST);
    } else {
        cv::resize(mat, buf, cv::Size(mat.cols * show_scale, mat.rows * show_scale), 0, 0, cv::INTER_NEAREST);
    }
    cv::imshow(name, buf);
    if (my_show_pos == -1) my_show_pos = show_pos;
    cv::setWindowTitle(name, my_show_pos == -1 ? cv::format("%s: live\n", name) : cv::format("%s: %d\n", name, my_show_pos));
}


int controldata = 0;//向下位机发送的打角行数据
int motordata = 28;//向下位机发送电机控制数据
uint8 sendmessage[14] = {0};//向下位机发送的数组
Mat datashow(CamH,CamW,CV_8UC3,Scalar(0,255,0));//显示数据
bool visionenable = false;//可视化使能标志位
void VisionData(String data,int x,int y);
void MyCVMatTounint8arry(Mat m1);
void Vision(void);
void getcontroldata_main(int SH,int SHR,int SM,int SMR,int SL);


pthread_mutex_t ai_lock;
pthread_cond_t ai_sig;
bool ai_flag, ai_finish_flag;
static void *ai_buf[60000];
int ai_len;
int ai_show_pos = -1;

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

        cv::Mat Aiframe(cv::Size(CAP_WIDTH, CAP_HEIGHT), CV_8UC3);//CAP_WIDTH,CAP_HEIGHT

        cv::imdecode(cv::Mat(1, ai_len, CV_8U, ai_buf), cv::IMREAD_COLOR, &Aiframe);

        // cv::resize(Aiframe, Aiframe, cv::Size(180, 140), 0, 0, cv::INTER_LINEAR);
        //Aiframe = Aiframe(cv::Rect(0, CAP_HEIGHT * 0.25, CAP_WIDTH, CAP_HEIGHT*0.75));

        cv::Point rookPoints[1][4];
        rookPoints[0][0] = cv::Point(0, 0);//左上角
        rookPoints[0][1] = cv::Point(319, 0);//右上角
        rookPoints[0][2] = cv::Point(319, CAP_HEIGHT * 0.05);//右下角
        rookPoints[0][3] = cv::Point(0, CAP_HEIGHT * 0.05);//左下角
        int npt[]={4};//二维数组 每列长度
        const cv::Point* ppt[1] = {rookPoints[0]};//所有多边形点坐标
        fillPoly(Aiframe, ppt, npt, 1, cv::Scalar(0, 0, 0));

        auto feeds = detection.preprocess(Aiframe, {320, 320});
        detection.run(*feeds);
        detection.render();  // get results

        // cv::Mat imageAi = Aiframe.clone();
        detection.drawBox(Aiframe);

        //my_image_show("AIimg", Aiframe, show_scale, ai_show_pos);//AI窗口
        if(visionenable){
          namedWindow("AiImage", WINDOW_NORMAL);
          imshow("AiImage",Aiframe);//
        }
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

  cout<<"首先进行配置文件读取"<<endl;
  std::ifstream in_file("/root/workspace/SDUWHSmartCar/config.json");
  nlohmann::json doc;
  if (!in_file.is_open()) {
    cout<<"打开配置文件失败！"<<endl;
    exit(1);
  }
  in_file >> doc;
  in_file.close();

  visionenable = doc["visionenable"];
  if(visionenable)
    cout<<"已开启可视化！"<<endl;
  else
    cout<<"可视化关闭！"<<endl;

  bool UartSendEnable = doc["UartSendEnable"];
  if(UartSendEnable)
    cout<<"已开启Uart发送！"<<endl;
  else
    cout<<"Uart发送关闭！"<<endl;

  bool EmergencyStop = doc["EmergencyStop"];
  if(EmergencyStop)
    cout<<"已开启紧急停车！"<<endl;
  else
    cout<<"紧急停车关闭！"<<endl;

  int HandThreshold = doc["HandThreshold"];
  cout<<"手动修正阈值HandThreshold = "<<HandThreshold<<endl;

  int SpeedHigh = doc["SpeedHigh"];
  int SpeedHighRange = doc["SpeedHighRange"];
  int SpeedMid = doc["SpeedMid"];
  int SpeedMidRange = doc["SpeedMidRange"];
  int SpeedLow = doc["SpeedLow"];
  cout<<"SpeedHigh = "<<SpeedHigh<<" SpeedHigh = "<<SpeedMid<<" SpeedLow = "<<SpeedLow<<endl;


  cout<<"配置文件读取完成!"<<endl;


  bool ret = cap_init(true, true, true);
  assert(!ret);

  // 初始化串口通信,新版本
  // ret = comm_init();
  // assert(!ret);
  if(UartSendEnable){
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    set_interface_attribs(fd, B460800);  
    set_mincount(fd, 0);
  }

  auto timer_start = std::chrono::steady_clock::now();

  pthread_t toAI;

  pthread_mutex_init(&ai_lock,NULL);

  pthread_create(&toAI,NULL,ai_thread,NULL);

  cout<<"初始化完成"<<endl;

 
    
  //局部变量声明区
  static bool is_first_frame = true;

  int Stopt = 0;
  int BlackCount = 0;
  int CountZebraStop = 80;
    
  //clock_t begin,end;//
  cout << "即将发车" << endl;

  while(true){

        
       
       
        cap_grab(buf, &len);
        if(is_first_frame){
            is_first_frame = false;
            continue;
        }
        
        assert(len < 60000);

        

        cv::Mat frame(cv::Size(CAP_WIDTH, CAP_HEIGHT), CV_8UC3);
        
        cv::imdecode(cv::Mat(1, len, CV_8U, buf), cv::IMREAD_COLOR, &frame);
            
        cv::resize(frame, frame, cv::Size(CamW, CamH), 0, 0, cv::INTER_LINEAR);

        //cvtColor(frame, frame, CV_BGR2GRAY);
        cv::Mat frame_splited[3];
        cv::split(frame, frame_splited);
        cv::Mat red_ch = frame_splited[2].clone();//
        cv::blur(red_ch, red_ch, cv::Size(3, 3)); // 减少噪点

        memcpy(images, red_ch.isContinuous() ? red_ch.data : red_ch.clone().data, sizeof images); // Mat 转数组




        //MyCVMatTounint8arry(frame);
        Threshold = otsuThreshold(images[0], CamW, CamH) + HandThreshold;
        Binaryisation(Threshold);

        ai_get_data_back();//

        RecYellowCone(frame);

        Tr_result.clear();
        for(PredictResult i : results){
          TrDetectResult temp;
          temp.x = (i.width/2 +i.x) * CamW / 320;
          temp.y = (i.height/2 +i.y) * CamH / 240;
          temp.type = i.type;
          temp.width = i.width * CamW / 320;
          temp.height = i.height * CamH / 240;
          Tr_result.push_back(temp);
        }

        Find_FengDing();//注意该函数必须在前面运行

        InitialiseFlag();

        scanbotton();

        getroadside();

        getmidline();

        getdistance();

        getlostside();

        //begin = clock(); 
        ReconElements();
        //end = clock();
        //cout<<"跑完一帧所用时间:"<<double(end - begin) / CLOCKS_PER_SEC * 1000<<"ms"<<endl;

        

        getmidline();

        getcontroldata_main(SpeedHigh,SpeedHighRange,SpeedMid,SpeedMidRange,SpeedLow);

        
       
        
        
        // uint8_t payload[10]; // 数据
        // // 串口通信数据发送
        // // TODO 对串口数据发送过程计时，评估对性能的影响
        // memset(payload, 0, sizeof payload);
        // payload[0] = 0X00;
        // payload[1] = 0X00; 
        // payload[2] = 0X00;
        // payload[3] = 0X00;
        // ret = comm_send_blocking(COMM_TYPE_UPDATE_TO_TC264, payload);
        // assert(!ret);
        if(UartSendEnable){
          //usb_uart_5byte_send_3_0(fd, sendmessage, motordata,-controldata);//
          usb_uart_send_4_0(fd, sendmessage, 200, 70, 50, 0.25);
        }

        if(EmergencyStop){

          if(BlackCount < 100)//
            BlackCount++;
          
          if(elementflag == InMainten || elementflag == OutMainten || BlackCount < 100)
            Stopt = 0;

          if(dofhead < 3){
            Stopt++;
            if(Stopt >= 35){
              cout<<"紧急停车！";
              break;
            }
          }else{
            Stopt = 0;
          }
        }
        

        if(elementflag == ZebraFlag){
          CountZebraStop--;
          if(CountZebraStop <= 0){
            cout<<"ZebraOut!"<<endl;
            break;
          }
        }
       
  
           
        if(visionenable){
          Vision();
         
          waitKey(1);
        }

        

    }
     cout<<"end"<<endl;

    return 0;
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
   VisionData("Topline:"+to_string(Topline),20,176);

  //  draw_point(rgbvision,rightchangepoint[0].x,rightchangepoint[0].y,CV_COLOR_YELLOW);
  //  draw_point(rgbvision,leftchangepoint[0].x,leftchangepoint[0].y,CV_COLOR_YELLOW);
  //  draw_point(rgbvision,rightchangepoint[1].x,rightchangepoint[1].y,CV_COLOR_CHOCOLATE);
  //  draw_point(rgbvision,leftchangepoint[1].x,leftchangepoint[1].y,CV_COLOR_CHOCOLATE);


  for(PredictResult i : results){
    i.x = (i.width/2 + i.x) * CamW / 320;
    i.y = (i.height/2 + i.y) * CamH / 240;
    //cout<<i.type<<" "<<i.label<<endl;//
    draw_point(rgbvision,i.x,i.y,CV_COLOR_SKYBLUE);
  }


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
  // namedWindow("GrayImage", WINDOW_NORMAL);
  cv::resize(rgbvision,rgbvision,cv::Size(rgbvision.cols*2, rgbvision.rows*2),0, 0, cv::INTER_NEAREST);
  imshow("GrayImage", rgbvision);


  // namedWindow("Data",WINDOW_NORMAL);
  cv::resize(datashow,datashow,cv::Size(datashow.cols*2, datashow.rows*2),0, 0, cv::INTER_NEAREST);
  imshow("Data",datashow);//
  
}

/*************************************************************************************************************************************/
void MyCVMatTounint8arry(Mat m1){
  for(int h = 0;h < CamH;h++){
    for(int w = 0;w < CamW;w++){
      images[h][w] = m1.at<uchar>(h,w);
    }
  }
}

void getcontroldata_main(int SH,int SHR,int SM,int SMR,int SL){

  getcontroldata(sendmessage,controldata,motordata,SH,SHR,SM,SMR,SL);
}