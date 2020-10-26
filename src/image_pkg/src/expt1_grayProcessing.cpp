#include <stdlib.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#define READIMAGE_ONLY
#ifndef READIMAGE_ONLY
#include <geometry_msgs/Twist.h>
#endif

using namespace cv;
using namespace std;



//直方图绘制函数，参数vector<int> nums 是灰度图片256级灰度的像素个数
void drawHist(vector<int> nums)
{
    Mat hist = Mat::zeros(600, 800, CV_8UC3);
    __gnu_cxx::__normal_iterator<int*, std::vector<int> >  Max = max_element(nums.begin(), nums.end()); //max迭代器类型,最大数目
    putText(hist, "Histogram", Point(150, 100), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255));
    //*********绘制坐标系************//
    Point o = Point(100, 550);
    Point x = Point(700, 550);
    Point y = Point(100, 150);
    //x轴
    line(hist, o, x, Scalar(255, 255, 255), 2, 8, 0);
    //y轴
    line(hist, o, y, Scalar(255, 255, 255), 2, 8, 0);

    //********绘制灰度曲线***********//
    Point pts[256];
    //生成坐标点
    for (int i = 0; i < 256; i++)
    {
        pts[i].x = i * 2 + 100;
        pts[i].y = 550 - int(nums[i] * (300.0 / (*Max))); //归一化到[0, 300]
        //显示横坐标
        if ((i + 1) % 16 == 0)
        {
            string num = format("%d", i + 1);
            putText(hist, num, Point(pts[i].x, 570), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        }
    }
    //绘制线
    for (int i = 1; i < 256; i++)
    {
        line(hist, pts[i - 1], pts[i], Scalar(0, 255, 0), 2);
    }
    //显示图像
    imshow("histogram", hist);
}
int main(int argc, char **argv)
{
	ROS_WARN("*****START*****");
	ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
	ros::NodeHandle n;

	//Before the use of camera, you can test ur program with images first: imread()
	VideoCapture capture;
	capture.open(0); //打开zed相机，如果要打开笔记本上的摄像头，需要改为0
	waitKey(100);
	if (!capture.isOpened())
	{
		printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
		return 0;
	}

#ifndef READIMAGE_ONLY
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义dashgo机器人的速度发布器
#endif

	Mat src_frame;
    Mat grey;
    int L=256;
	while (ros::ok())
	{
		capture.read(src_frame);
		if (src_frame.empty())
		{
			break;
		}
        
        //Equalization
        cvtColor(src_frame,grey,CV_BGR2GRAY);
        int M = grey.rows;
        int N = grey.cols;
		imshow("src", src_frame);
        imshow("grey",grey);
		vector<int> nums(256);
         for (int i = 0; i < M; i++)
        {
            //uchar *p = grey.ptr<uchar>(i);
             for (int j = 0; j < N; j++)
                 {
                    uchar value = grey.at<uchar>(i,j);
                    nums[(int)value]++;
                 }
        }
        
        
        //drawHist(nums);
        
        //long double Cr= (L-1)/(M*N);
        vector<double>  out(256);
        vector<int>  temp(256);
        for(int i=0;i<256;i++)
        {
            out[i]=0;
            temp[i]=0;
        }

        for(int i=0;i<256;i++)
        {
            double count=0;
            for(int j=0;j<=i;j++)
            {
                count+=255*nums[j]; 
            }
            out[i]=count;
            out[i]=out[i]/N;
            out[i]=out[i]/M;
            if(out[i]>255)
            {
                out[i]=255;
            }                       
           temp[i]=out[i]+0.5;
        }		
        
        //drawHist(temp);

        Mat Equlization = Mat(M,N,CV_8U);
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < N; j++) {
                int value2 = grey.at<uchar>(i,j);
                if(value2 > 255)
                    value2 = 255;
                Equlization.at<uchar>(i,j) = temp[value2];
            }
        }
        
        vector<int> nums_1(256);
         for (int i = 0; i < M; i++)
        {
            //uchar *p = grey.ptr<uchar>(i);
             for (int j = 0; j < N; j++)
                 {
                    uchar value = Equlization.at<uchar>(i,j);
                    nums_1[(int)value]++;
                 }
        }
        drawHist(nums_1);
        imshow("Equlization",Equlization);

		ros::spinOnce();
		waitKey(5);
	}
	return 0;
}




