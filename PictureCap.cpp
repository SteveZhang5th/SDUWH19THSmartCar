#include <opencv2/opencv.hpp>
#include <iostream>
#include "src/headfile.hpp"
#include "src/json.hpp"

using namespace cv;
using namespace std;

int main() {
    // // 打开默认的摄像头
    // VideoCapture cap(0);//

    // // 检查视频流是否打开
    // if (!cap.isOpened()) {
    //     cerr << "ERROR: Unable to open the camera" << endl;
    //     return -1;
    // }
    cout<<"首先进行配置文件读取"<<endl;
    std::ifstream in_file("/root/workspace/SDUWHSmartCar/config.json");
    nlohmann::json doc;
    if (!in_file.is_open()) {
        cout<<"打开配置文件失败！"<<endl;
        exit(1);
    }
    in_file >> doc;
    in_file.close();
    string Name = doc["capname"];

    Mat m1;
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


    // 定义一个计数器
    int count = 0;

    // 循环读取摄像头图像
    while (true) {
        // 获取一帧图像
        // Mat frame;
        // cap >> frame;
        // imshow("Frame", frame);

        vcap->getFrame((void **) &yuv422frame, (size_t *)&yuvframeSize);
        cvmat = cvMat(240,320,CV_8UC3,(void*)yuv422frame);	
        imgs = cvDecodeImage(&cvmat,1);
        m1 = cvarrToMat(imgs);
        //resize(m1, m1, Size(CamW, CamH));
        imshow("CapFrame", m1);

        // 按下“s”键保存图像
        char c = (char) waitKey(25);
        if (c == 's') {
            stringstream ss;
            ss << Name << count << ".jpg";
            imwrite(ss.str(), m1);
            cout << "Saved " << ss.str() << endl;
            count++;
        }

        // 按下ESC键退出
        if (c == 27) {
            break;
        }

        cvReleaseImage(&imgs);
        vcap->backFrame();
    }

    // 释放视频流
    //cap.release();

    // 关闭所有窗口
    destroyAllWindows();

    return 0;
}
