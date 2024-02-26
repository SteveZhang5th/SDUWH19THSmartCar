#include "src/headfile.hpp"
#include "src/json.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ctime>

using namespace std;
using namespace cv;
using json = nlohmann::json;

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



Mat m1;
Mat datashow(CamH,CamW,CV_8UC3,Scalar(0,255,0));//显示数据
void VisionData(String data,int x,int y);
void MyCVMatTounint8arry(Mat m1);
void Vision(void);
void getcontroldata(void);
string ImagesPath[5] = {"Bend","Cross","Fork","InRound","OutRound"};



int main(int argc, char *argv[]) {

    std::ifstream in_file("/root/workspace/SDUWHSmartCar/config.json");
    nlohmann::json doc;
    if (!in_file.is_open()) {
      cout<<"打开配置文件失败！"<<endl;
      exit(1);
    }
    in_file >> doc;
    in_file.close();

    if(doc["UartSendEnable"]){
      cout<<"booltest"<<endl;
    }


    int VisionEdge = 0;
    int pathnum = 0;
    bool CinEnable = false;
    while(!CinEnable){
        cout<<"0到16"<<endl;
        cout<<"请输入要静态测试的图像：";
        cin>>pathnum;

        if(pathnum >= 0 && pathnum <= 16)
            CinEnable = true;
        else
            cout<<"请输入已有编号！"<<endl;
    }
    cout<<"输入1开启传统边界可视化：";
    cin>>VisionEdge;
    if(VisionEdge == 1)
        MakeLineFlag = true;

/*****************************************************/
  //计时声明
  clock_t begin,end;


    cv::Mat img;
    //img = cv::imread("/home/edgeboard/workspace/SDUWHSmartCar/images/"+ImagesPath[pathnum]+".jpg",IMREAD_GRAYSCALE);
    img = cv::imread("/root/workspace/SDUWHSmartCar/build/Coneframe-"+to_string(pathnum)+".png");
    if(img.data == nullptr){
        cout<<"未能打开图片"<<endl;
        return 0;
    }

    

    resize(img, img, Size(CamW, CamH));
    

    cv::Mat frame_splited[3];
    cv::split(img, frame_splited);
    cv::Mat green_ch = frame_splited[1].clone(); // !!!!!!!现在green ch其实时红色通道的值，要改回来，将frame_splited[2] 改为 frame_splited[1] // opencv 为bgr 格式 即blue为[0] green为[1] red为[2]
    cv::Mat red_ch = frame_splited[2].clone();
    cv::Mat blue_ch = frame_splited[0].clone();
    namedWindow("green_ch", WINDOW_NORMAL);
    imshow("green_ch", green_ch);
    namedWindow("red_ch", WINDOW_NORMAL);
    imshow("red_ch", red_ch);
    namedWindow("blue_ch", WINDOW_NORMAL);
    imshow("blue_ch", blue_ch);
    cv::blur(red_ch, red_ch, cv::Size(3, 3)); // 减少噪点
    namedWindow("red_ch_after", WINDOW_NORMAL);
    imshow("red_ch_after", red_ch);

    // Mat HSV_yellow,mask_yellow;
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
    // //vector<vector<Point>> contours_blue;//蓝色轮廓寻找
    // //rectlist_blue.clear();
    // //Rect result(0,0,0,0);
    // Scalar Lower_yellow(26, 40, 46);
    // Scalar Upper_yellow(34, 255, 255);
    // Scalar Lower_blue(100, 43, 46);
    // Scalar Upper_blue(124, 255, 255);

    // cvtColor(img, HSV_yellow, COLOR_BGR2HSV);
    // namedWindow("HSV", WINDOW_NORMAL);
    // imshow("HSV", HSV_yellow);

    // inRange(HSV_yellow, Lower_yellow, Upper_yellow, mask_yellow);
    // namedWindow("mask_yellow", WINDOW_NORMAL);
    // imshow("mask_yellow", mask_yellow);
    
    //uint8 cone_img[CamH][CamW];
    //uint8 red_img[CamH][CamW];
    Mat Cone_mat = cv::Mat(CamH,CamW,CV_8UC1);

    
    
    for (int i = 0; i < img.rows; ++i) {
            cv::Vec3b *pixel = img.ptr<cv::Vec3b>(i); // point to first pixel in row
            for (int j = 0; j < img.cols; ++j) {

                // 绿-蓝
                int diff1 = pixel[j][1] - (int)pixel[j][0] * 1.1; //省赛1.1
                if (diff1 < 0) 
                    diff1 = 0;
                else if(diff1 > 2)
                    diff1 = 254;
                //cone_img[i][j] = diff1;//分离黄色
                Cone_mat.at<uchar>(i,j) = diff1;

                //红-绿
                // int diff2 = (int)pixel[j][2] - pixel[j][1];
                // if (diff2 < 0) 
                //     diff2 = 0;
                // else if(diff2 > 5)
                //     diff2 = 254;
                // red_img[i][j] = diff2;//分离红色
            }
      }

    

    //cv::Mat vision_cone_img = cv::Mat(CamH,CamW,CV_8UC1,cone_img);
    namedWindow("cone_org", WINDOW_NORMAL);
    imshow("cone_org", Cone_mat);
    //cv::Mat vision_red_img = cv::Mat(CamH,CamW,CV_8UC1,red_img);
    // erode(Cone_mat, Cone_mat, kernel_3, Point(-1, -1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());
    // namedWindow("cone_erode", WINDOW_NORMAL);
    // imshow("cone_erode", Cone_mat);
    dilate(Cone_mat, Cone_mat, kernel_3, Point(-1, -1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());
    namedWindow("cone_dilate", WINDOW_NORMAL);
    imshow("cone_dilate", Cone_mat);

    begin = clock();

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
      cout<<result.y<<" "<<result.area()<<endl;
      draw_point(img,result.x + result.width/2,result.y + result.height/2,CV_COLOR_GOLD);
      rectangle(img,result,CV_COLOR_RED,1);
    }

    end = clock();
    cout<<"跑完一帧所用时间:"<<double(end - begin) / CLOCKS_PER_SEC * 1000<<"ms"<<endl;

    namedWindow("img", WINDOW_NORMAL);
    imshow("img", img);
    // namedWindow("red", WINDOW_NORMAL);
    // imshow("red", vision_red_img);

    //cvtColor(img, img, CV_BGR2GRAY);

    //MyCVMatTounint8arry(img);
    memcpy(images, red_ch.isContinuous() ? red_ch.data : red_ch.clone().data, sizeof images); // Mat 转数组

    Threshold = otsuThreshold(images[0], CamW, CamH) - 4;
    Binaryisation(Threshold);

    Find_FengDing();

    InitialiseFlag();

    scanbotton();

    getroadside();

    getmidline();

    getdistance();

    getlostside();

    cout<<"LeftAngle:"<<endl;
    for(angelpoint an : LeftAngelList.List){
      cout<<an.orgpoint.angle<<" "<<an.orgpoint.x<<" "<<an.orgpoint.y<<" "<<an.angletype<<endl;
    }
    cout<<"RightAngle:"<<endl;
    for(angelpoint an : RightAngelList.List){
      cout<<an.orgpoint.angle<<" "<<an.orgpoint.x<<" "<<an.orgpoint.y<<" "<<an.angletype<<endl;
    }

    //ImagePerspective_Static();

    

    //getcontroldata();

    Vision();

    waitKey(0);


  
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

  datashow = Mat(CamH,CamW,CV_8UC3,Scalar(0,255,0));
  cv::Mat vision = cv::Mat(CamH,CamW,CV_8UC1,images);
  cv::Mat RV = cv::Mat(ReCamH,ReCamW,CV_8UC1,Reimages);
  cv::Mat rgbvision = cv::Mat(CamH,CamW,CV_8UC3);
  cvtColor(vision,rgbvision,CV_GRAY2BGR);
  

//   String yuzhi = to_string(Threshold);
  //VisionData(yuzhi,20,20);
   VisionData("dofhean:"+to_string(dofhead),20,20);
   VisionData("length_left:"+to_string(length_leftside),20,32);
   VisionData("length_right:"+to_string(length_rightside),20,44);
   VisionData("lostleftside:"+to_string(lostleftside),20,56);
   VisionData("lostrightside:"+to_string(lostrightside),20,68);
//   VisionData("elementflag:"+ElementString[(int)elementflag],20,80);
//   VisionData("controldata:"+to_string(controldata),20,92);//

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

  if(MakeLineFlag){
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
    MakeLineFlag = false;
    }

  // for (int i = CamH - 4; i > 2; i--) {
  //     rgbvision.at<Vec3b>(i,midline[i])[0] = 0;
  //     rgbvision.at<Vec3b>(i,midline[i])[1] = 255;
  //     rgbvision.at<Vec3b>(i,midline[i])[2] = 0;
  //  }
//   rgbvision.at<Vec3b>(waycontrolline1,Mid)[0] = 0;
//   rgbvision.at<Vec3b>(waycontrolline1,Mid)[1] = 0;
//   rgbvision.at<Vec3b>(waycontrolline1,Mid)[2] = 255;
//   rgbvision.at<Vec3b>(waycontrolline2,Mid)[0] = 0;
//   rgbvision.at<Vec3b>(waycontrolline2,Mid)[1] = 0;
//   rgbvision.at<Vec3b>(waycontrolline2,Mid)[2] = 255;
//   rgbvision.at<Vec3b>(waycontrolline3,Mid)[0] = 0;
//   rgbvision.at<Vec3b>(waycontrolline3,Mid)[1] = 0;
//   rgbvision.at<Vec3b>(waycontrolline3,Mid)[2] = 255;
    // for(int i = 0;i < 50;i++){//改为蓝色测试
    // rgbvision.at<Vec3b>(i,2)[0] = 255;
    // rgbvision.at<Vec3b>(i,2)[1] = 0;
    // rgbvision.at<Vec3b>(i,2)[2] = 0;
    // }
  namedWindow("GrayImage", WINDOW_NORMAL);
  imshow("GrayImage", rgbvision);

  // namedWindow("ReImage", WINDOW_NORMAL);
  // imshow("ReImage", RV);

  namedWindow("Data",WINDOW_NORMAL);
  imshow("Data",datashow);
  
}
